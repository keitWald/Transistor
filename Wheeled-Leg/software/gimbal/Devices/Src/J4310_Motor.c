#include "J4310_Motor.h"
#include <math.h>
#include "BSP_Can.h"
#include "BSP_Lock.h"
#include "Gimbal_Config.h"
#include <string.h>
static volatile J4310_t pitch;
_Static_assert(GIMBAL_PITCH_TX_ID <= 0x7FF && GIMBAL_PITCH_RX_ID <= 0x7FF, "Standard CAN IDs");

static float decode(uint32_t value, float maximum, uint32_t bits)
{
    return (float)value * (2.0f * maximum) / (float)((1U << bits) - 1U) - maximum;
}
void J4310_Receive(uint16_t id, const uint8_t data[8])
{
    if (id != GIMBAL_PITCH_RX_ID) return;
    J4310_t value = {0};
    uint32_t p = (uint32_t)data[1] << 8 | data[2];
    uint32_t v = (uint32_t)data[3] << 4 | data[4] >> 4;
    uint32_t t = (uint32_t)(data[4] & 0x0FU) << 8 | data[5];
    value.position = decode(p, GIMBAL_DM_P_MAX, 16);
    value.velocity = decode(v, GIMBAL_DM_V_MAX, 12);
    value.torque = decode(t, GIMBAL_DM_T_MAX, 12);
    value.state = data[0] >> 4;
    value.mos_temperature = data[6]; value.rotor_temperature = data[7];
    value.last_ms = HAL_GetTick(); value.seen = 1;
    pitch = value;
}
void J4310_Get(J4310_t *motor)
{
    uint32_t mask = BSP_Lock();
    *motor = pitch;
    BSP_Unlock(mask);
}
static uint32_t encode(float value, float minimum, float maximum, uint32_t bits)
{
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    return (uint32_t)((value - minimum) * (float)((1U << bits) - 1U) / (maximum - minimum));
}
int J4310_PackMIT(uint8_t data[8], float position, float velocity,
                     float kp, float kd, float torque)
{
    if (data == NULL || !isfinite(position) || !isfinite(velocity) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(torque)) return 0;
    uint32_t p = encode(position, -GIMBAL_DM_P_MAX, GIMBAL_DM_P_MAX, 16);
    uint32_t v = encode(velocity, -GIMBAL_DM_V_MAX, GIMBAL_DM_V_MAX, 12);
    uint32_t k = encode(kp, 0, 500, 12);
    uint32_t d = encode(kd, 0, 5, 12);
    uint32_t t = encode(torque, -GIMBAL_DM_T_MAX, GIMBAL_DM_T_MAX, 12);
    data[0] = (uint8_t)(p >> 8); data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)((v & 15U) << 4 | k >> 8);
    data[4] = (uint8_t)k; data[5] = (uint8_t)(d >> 4);
    data[6] = (uint8_t)((d & 15U) << 4 | t >> 8);
    data[7] = (uint8_t)t;
    return 1;
}
HAL_StatusTypeDef J4310_Send(float position, float velocity,
                                   float kp, float kd, float torque)
{
    uint8_t data[8];
    if (!J4310_PackMIT(data, position, velocity, kp, kd, torque)) return HAL_ERROR;
    return CAN_SendData(&hfdcan2, GIMBAL_PITCH_TX_ID, data);
}
HAL_StatusTypeDef J4310_Special(uint8_t command)
{
    uint8_t data[8];
    if (command != 0xFCU && command != 0xFDU) return HAL_ERROR;
    memset(data, 0xFF, sizeof(data)); data[7] = command;
    return CAN_SendData(&hfdcan2, GIMBAL_PITCH_TX_ID, data);
}

