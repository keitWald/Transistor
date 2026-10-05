#include "BSP_BoardCommunication.h"
#include "BSP_Lock.h"
#include "Gimbal_Config.h"
#include "SBUS.h"
#include <math.h>
#include <string.h>

ControlMessge ControlMes;
static volatile uint32_t last_rx_ms;
static volatile uint8_t rx_seen;
_Static_assert(sizeof(float) == 4 && sizeof(double) == 8, "Existing board protocol float widths");

static int16_t read_be16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}
static void write_be16(uint8_t *p, int16_t value)
{
    p[0] = (uint8_t)((uint16_t)value >> 8); p[1] = (uint8_t)value;
}
void Board1_To_2(void)
{
    ControlMessge value;
    SBUS_RevPack_t remote;
    SBUS_Get(&remote);
    uint32_t mask = BSP_Lock();
    value = ControlMes;
    BSP_Unlock(mask);
    if (!remote.armed) {
        value.x_velocity = value.y_velocity = value.z_rotation_velocity = value.yaw_velocity = 0;
        value.fric_Flag = value.AutoAimFlag = value.NUC_AutoAimFlag = 0;
        value.change_Flag = value.reset_Flag = value.shoot_Speed = value.yaw_torque_only_mode = 0;
        value.yaw_torque = value.yaw_target_velocity = 0;
    }
    uint8_t data[8] = {0};
    write_be16(data, value.x_velocity); 
    write_be16(data + 2, value.y_velocity);
    write_be16(data + 4, value.z_rotation_velocity); 
    write_be16(data + 6, value.yaw_velocity);
    (void)CAN_SendData(&hfdcan3, CAN_ID_CHASSIS, data);

    memset(data, 0, sizeof(data));
    write_be16(data, value.yaw_position);
    data[2] = value.shoot_Speed;
    data[3] = (uint8_t)((value.fric_Flag & 1U) | (value.AutoAimFlag & 1U) << 1 |
               (value.change_Flag & 1U) << 2 | (value.reset_Flag & 1U) << 3);
    data[4] = value.modelFlag; data[5] = value.NUC_AutoAimFlag & 1U;
    data[6] = value.yaw_torque_only_mode & 1U;
    (void)CAN_SendData(&hfdcan3, CAN_ID_GIMBAL, data);

    memcpy(data, &value.yaw_target_angle, 8);
    (void)CAN_SendData(&hfdcan3, CAN_ID_TARGETANGLE, data);
    memcpy(data, &value.yaw_torque, 4); memcpy(data + 4, &value.yaw_target_velocity, 4);
    (void)CAN_SendData(&hfdcan3, CAN_ID_TORQUE, data);
}
void Board1_getGimbalInfo(Can_Export_Data_t frame)
{
    const uint8_t *data = frame.CANx_Export_RxMessage;
    ControlMes.yaw_realAngle = read_be16(data);
    ControlMes.Blood_Volume = (uint16_t)read_be16(data + 2);
    ControlMes.Speed_Bullet = (float)read_be16(data + 4) / 1000.0f;
    ControlMes.tnndcolor = data[6] & 1U;
    ControlMes.game_start = ((frame.StdId == CAN_ID_CHASSIS ? data[7] : data[6]) >> 1) & 1U;
}
void Board1_getChassisIMU(Can_Export_Data_t frame)
{
    const uint8_t *data = frame.CANx_Export_RxMessage;
    ControlMes.chassis_imu_yaw = (float)read_be16(data) / 1000.0f;
    ControlMes.chassis_imu_omega = (float)read_be16(data + 2) / 1000.0f;
    ControlMes.yaw_omega = (float)read_be16(data + 4) / 1000.0f;
    ControlMes.chassis_yaw_temp = data[6];
}
void Board1_get_yawangle_double(Can_Export_Data_t frame)
{
    double angle;
    memcpy(&angle, frame.CANx_Export_RxMessage, 8);
    if (isfinite(angle)) ControlMes.yaw_realAngle_double = angle;
}
void Board1_Receive(uint16_t id, const uint8_t data[8])
{
    Can_Export_Data_t frame;
    frame.StdId = id; memcpy(frame.CANx_Export_RxMessage, data, 8);
    switch (id) {
    case CAN_ID_CHASSIS:
    case CAN_ID_GIMBAL: Board1_getGimbalInfo(frame); break;
    case CAN_ID_CHASSIS_IMU: Board1_getChassisIMU(frame); break;
    case CAN_ID_DOUBLE_YAWANGLE: Board1_get_yawangle_double(frame); break;
    default: return;
    }
    rx_seen = 1; last_rx_ms = HAL_GetTick();
}
uint8_t Board1_IsOnline(void)
{
    uint32_t mask = BSP_Lock();
    uint8_t online = (uint8_t)(rx_seen &&
        (uint32_t)(HAL_GetTick() - last_rx_ms) <= GIMBAL_BOARD_TIMEOUT_MS);
    BSP_Unlock(mask);
    return online;
}


