#include "M3508_Motor.h"
/* Adapted from the existing M3508 feedback and C620 group-current layout. */
#include "BSP_Can.h"
#include "BSP_Lock.h"
#include "Gimbal_Config.h"
#include <string.h>
static volatile M3508_t friction[2];
_Static_assert(GIMBAL_FRIC_LEFT_ID >= 0x205 && GIMBAL_FRIC_LEFT_ID <= 0x208, "C620 group 0x1FF");
_Static_assert(GIMBAL_FRIC_RIGHT_ID >= 0x205 && GIMBAL_FRIC_RIGHT_ID <= 0x208, "C620 group 0x1FF");
_Static_assert(GIMBAL_FRIC_LEFT_ID != GIMBAL_FRIC_RIGHT_ID, "Distinct friction IDs");
void M3508_Receive(uint16_t id, const uint8_t data[8])
{
    int index;
    if (id == GIMBAL_FRIC_LEFT_ID) index = 0;
    else if (id == GIMBAL_FRIC_RIGHT_ID) index = 1;
    else return;
    M3508_t value = {0};
    value.encoder = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    if (value.encoder >= 8192U) return;
    value.rpm = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    value.current = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    value.temperature = data[6];
    value.last_ms = HAL_GetTick();
    value.seen = 1;
    friction[index] = value;
}
void M3508_Get(M3508_t motors[2])
{
    uint32_t mask = BSP_Lock();
    motors[0] = friction[0]; motors[1] = friction[1];
    BSP_Unlock(mask);
}
HAL_StatusTypeDef M3508_Send(int16_t left, int16_t right)
{
    uint8_t data[8] = {0};
    if (left > 16384) left = 16384;
    if (left < -16384) left = -16384;
    if (right > 16384) right = 16384;
    if (right < -16384) right = -16384;
    unsigned l = (GIMBAL_FRIC_LEFT_ID - 0x205U) * 2U;
    unsigned r = (GIMBAL_FRIC_RIGHT_ID - 0x205U) * 2U;
    data[l] = (uint8_t)((uint16_t)left >> 8); data[l + 1U] = (uint8_t)left;
    data[r] = (uint8_t)((uint16_t)right >> 8); data[r + 1U] = (uint8_t)right;
    return CAN_SendData(&hfdcan1, GIMBAL_FRIC_COMMAND_ID, data);
}


