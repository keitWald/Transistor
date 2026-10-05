#ifndef BSP_BOARDCOMMUNICATION_H
#define BSP_BOARDCOMMUNICATION_H
#include "BSP_Can.h"
#define CAN_ID_CHASSIS      0x10FU
#define CAN_ID_GIMBAL       0x11FU
#define CAN_ID_TORQUE       0x12FU
#define CAN_ID_CHASSIS_IMU  0x13FU
#define CAN_ID_DOUBLE_YAWANGLE 0x14FU
#define CAN_ID_TARGETANGLE  0x15FU
#define FDCAN_ID_IT_KEYCOMMAND 0x22FU
#define model_Normal 0U
#define model_Record 1U
#define model_Follow 2U
/* Preserve existing application field names and wire format. */
typedef struct {
    int16_t x_velocity, y_velocity, z_rotation_velocity, pitch_velocity, yaw_velocity;
    int16_t yaw_position;
    uint8_t AutoAimFlag, NUC_AutoAimFlag, shoot_state;
    int16_t yaw_realAngle;
    float Speed_Bullet;
    int16_t heat_remain;
    uint8_t modelFlag, shoot_Speed, change_Flag, fric_Flag, reset_Flag, tnndcolor;
    uint8_t redial, jump_Flag;
    uint16_t Blood_Volume, game_start;
    float yaw_torque, yaw_omega, chassis_imu_yaw, chassis_imu_omega;
    uint8_t chassis_yaw_temp;
    float Auto_Aim_Pitch;
    uint8_t yaw_torque_only_mode;
    float yaw_target_velocity;
    double yaw_realAngle_double, yaw_target_angle;
} ControlMessge;
extern ControlMessge ControlMes;
void Board1_To_2(void);
void Board1_Receive(uint16_t id, const uint8_t data[8]);
uint8_t Board1_IsOnline(void);
void Board1_getGimbalInfo(Can_Export_Data_t frame);
void Board1_getChassisIMU(Can_Export_Data_t frame);
void Board1_get_yawangle_double(Can_Export_Data_t frame);
#endif

