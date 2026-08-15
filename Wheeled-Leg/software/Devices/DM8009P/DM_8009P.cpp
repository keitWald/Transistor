/**
 * @file DM_8009P.cpp
 * @author ZHY
 * @brief 达妙电机配置与操作
 * @version
 * @date
 *
 * @copyright
 *
 */
#include "DM_8009P.h"
#include "BSP_fdcan.h"
#include "crc.h"
#include "user_lib.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
/**
 * @brief  uint类型转换为float类型
 * @param
 * @retval None
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits) {
  float span = X_max - X_min;
  float offset = X_min;
  return ((float)X_int) * span / ((float)((1 << Bits) - 1)) + offset;
}

/**
 * @brief  float类型转换为uint类型
 * @param
 * @retval None
 */
static int float_to_uint(float X_float, float X_min, float X_max, int bits) {
  if (X_float < X_min) X_float = X_min;
  if (X_float > X_max) X_float = X_max;
  float span = X_max - X_min;
  float offset = X_min;
  return (int)((X_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief  使能8009电机
 * @param
 * @retval None
 */
void Class_Motor_DM_8009P::Enable() {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;
  Can_Fun.fdcanx_send_data(Can_DM_Motor, CAN_Tx_ID, data, 8);
}

/**
 * @brief  重新设置8009P电机零点
 * @param
 * @retval None
 */
void Class_Motor_DM_8009P::Save_Pos_Zero() {
  uint8_t data[8];
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFE;
  Can_Fun.fdcanx_send_data(Can_DM_Motor, CAN_Tx_ID, data, 8);
}
/**
 * @brief 电机初始化
 *
 * @param hcan 绑定的CAN总线
 * @param _CAN_Rx_ID 收数据绑定的CAN ID, 与上位机驱动参数Master_ID保持一致,
 * 传统模式有效
 * @param _CAN_Tx_ID 发数据绑定的CAN ID,
 * 是上位机驱动参数CAN_ID加上控制模式的偏移量, 传统模式有效
 * @param _Motor_DM_Control_Method 电机控制方式
 * @param _Angle_Max 最大位置, 与上位机控制幅值PMAX保持一致, 传统模式有效
 * @param _Omega_Max 最大速度, 与上位机控制幅值VMAX保持一致, 传统模式有效
 * @param _Torque_Max 最大扭矩, 与上位机控制幅值TMAX保持一致, 传统模式有效
 */
void Class_Motor_DM_8009P::Init(
    FDCAN_HandleTypeDef *hfdcan, uint8_t _CAN_Rx_ID, uint8_t _CAN_Tx_ID,
    Enum_Motor_DM_Control_Method _Motor_DM_Control_Method, float _Angle_Max,
    float _Omega_Max, float _Torque_Max, float _Current_Max) {
  Can_DM_Motor = hfdcan;
  CAN_Rx_ID = _CAN_Rx_ID;
  CAN_Tx_ID = _CAN_Tx_ID;
  switch (_Motor_DM_Control_Method) {
  case (Motor_DM_Control_Method_NORMAL_MIT): {
    CAN_Tx_ID = _CAN_Tx_ID;
    break;
  }

  case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA): {
    CAN_Tx_ID = _CAN_Tx_ID + 0x100;
    break;
  }
  case (Motor_DM_Control_Method_NORMAL_OMEGA): {
    CAN_Tx_ID = _CAN_Tx_ID + 0x200;
    break;
  }
  }
  Motor_DM_Control_Method = _Motor_DM_Control_Method;
  Angle_Max = _Angle_Max;
  Omega_Max = _Omega_Max;
  Torque_Max = _Torque_Max;
  Current_Max = _Current_Max;
}

/**
 * @brief  从CAN报文中获取DM_8009P电机信息
 * @param  RxMessage 	CAN报文接收结构体
 * @retval None
 */
void Class_Motor_DM_8009P::DM_8009P_getInfo(FDCan_Export_Data_t RxMessage) {
  // 检查ID是否匹配
  if (RxMessage.fdcan_RxHeader.Identifier != CAN_Rx_ID) {
    return;
  }
  Flag++;
  DM_Rev.id = RxMessage.fdcan_RxHeader.Identifier;
  DM_Rev.last_pos = DM_Rev.Now_pos;
  DM_Rev.state = RxMessage.FDCANx_Export_RxMessage[0] >> 4;
  DM_Rev.Receive_pos = (uint16_t)((RxMessage.FDCANx_Export_RxMessage[1] << 8) |
                                  RxMessage.FDCANx_Export_RxMessage[2]);
  DM_Rev.Receive_vel = (uint16_t)((RxMessage.FDCANx_Export_RxMessage[3] << 4) |
                                  (RxMessage.FDCANx_Export_RxMessage[4] >> 4));
  DM_Rev.Receive_torq =
      (uint16_t)(((RxMessage.FDCANx_Export_RxMessage[4] & 0x0F) << 8) |
                 RxMessage.FDCANx_Export_RxMessage[5]);
  DM_Rev.Now_pos = uint_to_float(DM_Rev.Receive_pos, -P_MAX, P_MAX, 16);
  DM_Rev.Now_vel = uint_to_float(DM_Rev.Receive_vel, -V_MAX, V_MAX, 12);
  DM_Rev.Now_torq = uint_to_float(DM_Rev.Receive_torq, -T_MAX, T_MAX, 12);
  DM_Rev.Now_Tmos = (float)(RxMessage.FDCANx_Export_RxMessage[6]);
  DM_Rev.Now_Trotor = (float)(RxMessage.FDCANx_Export_RxMessage[7]);

  if (DM_Rev.Now_pos - DM_Rev.last_pos > -6500) {
    turnCount++;
  }
  if (DM_Rev.Now_pos - DM_Rev.last_pos < -6500) {
    turnCount--;
  }
  totalpos = DM_Rev.Now_pos + (8192 * turnCount);
  // 帧率统计，数据更新标志位
  InfoUpdateFrame++;
  InfoUpdateFlag = 1;
}

/**
 * @brief 通过can总线发送MIT模式下的控制帧
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
void Class_Motor_DM_8009P::DM_8009P_Ctrl() {
  TIM_Alive_PeriodElapsedCallback();
  uint8_t data[8] = {0};
  uint8_t len = 0;
  bool send = false;
  switch (Motor_DM_Control_Method) {
  case Motor_DM_Control_Method_NORMAL_MIT:
    send = BuildMitFrame(data, &len);
    break;
  case Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA:
    send = BuildAngleOmegaFrame(data, &len);
    break;
  case Motor_DM_Control_Method_NORMAL_OMEGA:
    send = BuildOmegaFrame(data, &len);
    break;
  default:
    send = false;
    break;
  }
  if (send) {
    Can_Fun.fdcanx_send_data(Can_DM_Motor, CAN_Tx_ID, data, len);
  }
}

/**
 * @brief 获取电机状态
 *
 * @return Enum_Motor_DM_Status 电机状态
 */
Enum_Motor_DM_Status Class_Motor_DM_8009P::Get_Status() { return (Status); }

void Class_Motor_DM_8009P::Set_Target_Pos(float _pos) {
  DM_Mit.target_pos = _pos;
}

void Class_Motor_DM_8009P::Set_Target_Vel(float _vel) {
  DM_Mit.target_vel = _vel;
}

void Class_Motor_DM_8009P::Set_Target_Tor(float _torq) {
  DM_Mit.target_torq = _torq;
}

void Class_Motor_DM_8009P::Set_Target_Kp(float _kp) { DM_Mit.kp = _kp; }

void Class_Motor_DM_8009P::Set_Target_Kd(float _kd) { DM_Mit.kd = _kd; }

void Class_Motor_DM_8009P::Set_MIT(float _pos, float _vel, float _kp, float _kd,
                                   float _tff) {
  if (Motor_DM_Control_Method != Motor_DM_Control_Method_NORMAL_MIT) {
    return;
  }
  Set_Target_Pos(_pos);
  Set_Target_Vel(_vel);
  Set_Target_Kp(_kp);
  Set_Target_Kd(_kd);
  Set_Target_Tor(_tff);
}

void Class_Motor_DM_8009P::Set_PosVel(float _pos, float _vel) {
  if (Motor_DM_Control_Method != Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA) {
    return;
  }
  Set_Target_Pos(_pos);
  Set_Target_Vel(_vel);
}

void Class_Motor_DM_8009P::Set_Vel(float _vel) {
  if (Motor_DM_Control_Method != Motor_DM_Control_Method_NORMAL_OMEGA) {
    return;
  }
  Set_Target_Vel(_vel);
}

bool Class_Motor_DM_8009P::BuildMitFrame(uint8_t data[8], uint8_t *len) {
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  float motor_pos = DM_Mit.target_pos;
  float motor_vel = DM_Mit.target_vel;
  pos_tmp = float_to_uint(motor_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(motor_vel, V_MIN, V_MAX, 12);
  kp_tmp = float_to_uint(DM_Mit.kp, KP_MIN, KP_MAX, 12);
  kd_tmp = float_to_uint(DM_Mit.kd, KD_MIN, KD_MAX, 12);
  tor_tmp = float_to_uint(DM_Mit.target_torq, T_MIN, T_MAX, 12);
  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;
  *len = 8;
  return true;
}

bool Class_Motor_DM_8009P::BuildAngleOmegaFrame(uint8_t data[8], uint8_t *len) {
  const float motor_pos = DM_Mit.target_pos;
  const float motor_vel = DM_Mit.target_vel;
  const uint8_t *pbuf = reinterpret_cast<const uint8_t *>(&motor_pos);
  const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&motor_vel);

  data[0] = pbuf[0];
  data[1] = pbuf[1];
  data[2] = pbuf[2];
  data[3] = pbuf[3];
  data[4] = vbuf[0];
  data[5] = vbuf[1];
  data[6] = vbuf[2];
  data[7] = vbuf[3];
  *len = 8;
  return true;
}

bool Class_Motor_DM_8009P::BuildOmegaFrame(uint8_t data[8], uint8_t *len) {
  const float motor_vel = DM_Mit.target_vel;
  const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&motor_vel);

  data[0] = vbuf[0];
  data[1] = vbuf[1];
  data[2] = vbuf[2];
  data[3] = vbuf[3];
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  *len = 4;
  return true;
}
float Class_Motor_DM_8009P::GetAngle() { return (DM_Rev.Now_pos); }
float Class_Motor_DM_8009P::GetSpeed() { return (DM_Rev.Now_vel); }
float Class_Motor_DM_8009P::GetTor() { return (DM_Rev.Now_torq); }

// 定时检测达妙电机是否存活
void Class_Motor_DM_8009P::TIM_Alive_PeriodElapsedCallback() {
  // Feedback does not necessarily arrive in every 1 ms control slot.  A
  // single missed slot must not disable/re-enable the motor; only declare it
  // offline after a bounded continuous gap.
  if (Flag == Pre_Flag) {
    if (feedback_miss_ticks_ < DM_8009P_FEEDBACK_TIMEOUT_TICKS) {
      feedback_miss_ticks_++;
    }
    if (feedback_miss_ticks_ >= DM_8009P_FEEDBACK_TIMEOUT_TICKS) {
      Status = Motor_DM_Status_DISABLE;
    }
  } else {
    feedback_miss_ticks_ = 0U;
    Status = Motor_DM_Status_ENABLE;
  }
  Pre_Flag = Flag;
  // Do not auto-enable here.  Fault/offline recovery is owned by the chassis
  // state machine and requires an operator-confirmed ESTOP -> RECOVER cycle.
}
