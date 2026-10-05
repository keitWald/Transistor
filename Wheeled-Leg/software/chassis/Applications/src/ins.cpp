/* Includes ------------------------------------------------------------------*/
#include "ins.h"
#include "bsp_dwt.h"
#include "pid.h"
#include "tim.h"
#include "user_lib.h"
#include "N100.h"
#include <math.h>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
INS_t INS;
/* Private function prototypes -----------------------------------------------*/
#define X 0
#define Y 1
#define Z 2
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};


uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;

static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]);
static void NormalizeQuaternion(float q[4]);

static float LimitLeverArmYawRate(float yaw_rate) {
  if (!isfinite(yaw_rate))
    return 0.0f;
  if (yaw_rate > INS_LEVER_ARM_YAW_RATE_MAX)
    return INS_LEVER_ARM_YAW_RATE_MAX;
  if (yaw_rate < -INS_LEVER_ARM_YAW_RATE_MAX)
    return -INS_LEVER_ARM_YAW_RATE_MAX;
  return yaw_rate;
}

float INS_CompensatePitchYawLeverArm(float raw_pitch, float yaw_rate,
                                    float forward_offset_m) {
  if (!isfinite(raw_pitch) || !isfinite(forward_offset_m))
    return raw_pitch;
  const float limited_yaw_rate = LimitLeverArmYawRate(yaw_rate);
  const float centripetal_acceleration =
      forward_offset_m * limited_yaw_rate * limited_yaw_rate;
  const float apparent_pitch =
      atan2f(centripetal_acceleration, STANDARD_GRAVITY);
  return raw_pitch - apparent_pitch;
}

float INS_CompensateRollYawLeverArm(float raw_roll, float yaw_rate,
                                   float lateral_offset_m) {
  if (!isfinite(raw_roll) || !isfinite(lateral_offset_m))
    return raw_roll;
  const float limited_yaw_rate = LimitLeverArmYawRate(yaw_rate);
  const float centripetal_acceleration =
      lateral_offset_m * limited_yaw_rate * limited_yaw_rate;
  const float apparent_roll =
      atan2f(centripetal_acceleration, STANDARD_GRAVITY);
  return raw_roll - apparent_roll;
}



void INS_Init(void) {
#if INS_IMU_SOURCE == INS_IMU_SOURCE_N100
  N100_Init();
#elif INS_IMU_SOURCE == INS_IMU_SOURCE_DM_IMU
  dm_imu_init();
#else
#error "INS_IMU_SOURCE is invalid"
#endif
}

void INS_Task(void) {

#if INS_IMU_SOURCE == INS_IMU_SOURCE_N100
  const float gravity[3] = {0, 0, 9.805f};
#elif INS_IMU_SOURCE == INS_IMU_SOURCE_DM_IMU
  const float gravity[3] = {0, 0, 9.805f};
#else
#error "INS_IMU_SOURCE is invalid"
#endif
  dt = DWT_GetDeltaT(&INS_DWT_Count);
  t += dt;

#if INS_IMU_SOURCE == INS_IMU_SOURCE_N100
  N100_Read();
  INS.Accel[X] = IMUData_Packet.accelerometer_x;
  INS.Accel[Y] = IMUData_Packet.accelerometer_y;
  INS.Accel[Z] = IMUData_Packet.accelerometer_z;

  INS.Gyro[X] = IMUData_Packet.gyroscope_x;
  INS.Gyro[Y] = IMUData_Packet.gyroscope_y;
  INS.Gyro[Z] = IMUData_Packet.gyroscope_z;

  INS.q[0] = AHRSData_Packet.Qw;
  INS.q[1] = AHRSData_Packet.Qx;
  INS.q[2] = AHRSData_Packet.Qy;
  INS.q[3] = AHRSData_Packet.Qz;
  NormalizeQuaternion(INS.q);
  BodyFrameToEarthFrame(xb, INS.xn, INS.q);
  BodyFrameToEarthFrame(yb, INS.yn, INS.q);
  BodyFrameToEarthFrame(zb, INS.zn, INS.q);
  INS.Roll = AHRSData_Packet.Roll;
  INS.Pitch = AHRSData_Packet.Pitch;
  INS.Yaw = AHRSData_Packet.Heading;
  INS.YawSpeed = AHRSData_Packet.HeadingSpeed;
#else
  // DM-IMU-L1 serial output already uses m/s^2 for acceleration.
  INS.Accel[X] = dm_imu.accel[0];
  INS.Accel[Y] = dm_imu.accel[1];
  INS.Accel[Z] = dm_imu.accel[2];

  // DM-IMU-L1 serial angular-rate output is already rad/s on every axis.
  // Empirical X/Y rate gains are migrated in Chassis.h; model-based LQR/VMC/
  // LESO paths and physical rate thresholds consume these SI values directly.
  INS.Gyro[X] = dm_imu.gyro[0];
  INS.Gyro[Y] = dm_imu.gyro[1];
  INS.YawSpeed = INS.Gyro[Z] = dm_imu.gyro[2];

  INS.q[0] = dm_imu.quaternion[0];
  INS.q[1] = dm_imu.quaternion[1];
  INS.q[2] = dm_imu.quaternion[2];
  INS.q[3] = dm_imu.quaternion[3];
  NormalizeQuaternion(INS.q);
  BodyFrameToEarthFrame(xb, INS.xn, INS.q);
  BodyFrameToEarthFrame(yb, INS.yn, INS.q);
  BodyFrameToEarthFrame(zb, INS.zn, INS.q);
  const float raw_roll = dm_imu.roll * DEGREE_2_RAD;
  const float raw_pitch = dm_imu.pitch * DEGREE_2_RAD;
  INS.Roll = raw_roll;
  INS.Pitch = INS_CompensatePitchYawLeverArm(
      raw_pitch, INS.YawSpeed, INS_IMU_FORWARD_LEVER_ARM_M);
  INS.Yaw = dm_imu.yaw * DEGREE_2_RAD;
#endif

  float gravity_b[3];
  EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
  for (uint8_t i = 0; i < 3; i++) {
    INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * 0.9 +
                           INS.MotionAccel_b[i] * 0.1;
  }
  BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

  if (INS.time > 3000) {
    INS.flag = 1;   // Wait 3s after boot.
  } else {
    INS.time++;
  }
}

static void NormalizeQuaternion(float q[4]) {
  const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                           q[3] * q[3]);
  if (norm > 1e-6f) {
    const float inv = 1.0f / norm;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
  }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                     (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                     (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, const float* q) {
    // 使用原始的、未经修改的函数体
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
/**
 * @brief
 * reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
      fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[X] = c_11 * gyro_temp[X] + c_12 * gyro_temp[Y] + c_13 * gyro_temp[Z];
  gyro[Y] = c_21 * gyro_temp[X] + c_22 * gyro_temp[Y] + c_23 * gyro_temp[Z];
  gyro[Z] = c_31 * gyro_temp[X] + c_32 * gyro_temp[Y] + c_33 * gyro_temp[Z];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    accel_temp[i] = accel[i];

  accel[X] = c_11 * accel_temp[X] + c_12 * accel_temp[Y] + c_13 * accel_temp[Z];
  accel[Y] = c_21 * accel_temp[X] + c_22 * accel_temp[Y] + c_23 * accel_temp[Z];
  accel[Z] = c_31 * accel_temp[X] + c_32 * accel_temp[Y] + c_33 * accel_temp[Z];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}
/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll) {
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
         57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                  2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
           57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float* q) {
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = arm_cos_f32(Pitch / 2);
  cosYaw = arm_cos_f32(Yaw / 2);
  cosRoll = arm_cos_f32(Roll / 2);
  sinPitch = arm_sin_f32(Pitch / 2);
  sinYaw = arm_sin_f32(Yaw / 2);
  sinRoll = arm_sin_f32(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
