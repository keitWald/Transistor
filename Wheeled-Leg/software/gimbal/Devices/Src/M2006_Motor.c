/* Adapted from the reference M2006_getInfo/C610 group-current layout. */
#include "M2006_Motor.h"
#include "Gimbal_Config.h"
#include "BSP_Can.h"
#include "BSP_Lock.h"
static volatile M2006_t motor;
_Static_assert(GIMBAL_DIAL_RX_ID >= 0x201 && GIMBAL_DIAL_RX_ID <= 0x204, "C610 0x200 group");
_Static_assert(GIMBAL_DIAL_RX_ID != GIMBAL_PITCH_RX_ID &&
               GIMBAL_DIAL_RX_ID != GIMBAL_PITCH_TX_ID, "Distinct FDCAN2 motor IDs");
void M2006_Receive(uint16_t id, const uint8_t data[8])
{
    if (id != GIMBAL_DIAL_RX_ID) return;
    uint16_t encoder = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    if (encoder >= 8192U) return;
    uint32_t now = HAL_GetTick();
    if (!motor.seen || (uint32_t)(now - motor.last_ms) > GIMBAL_MOTOR_TIMEOUT_MS) {
        motor.continuity++; /* Do not infer turns through a feedback outage. */
    } else {
        int32_t delta = (int32_t)encoder - motor.encoder;
        if (delta > 4096) delta -= 8192;
        if (delta < -4096) delta += 8192;
        motor.total_ticks += delta;
    }
    motor.encoder = encoder;
    motor.rpm = (int16_t)((uint16_t)data[2] << 8 | data[3]);
    motor.current = (int16_t)((uint16_t)data[4] << 8 | data[5]);
    motor.last_ms = now; motor.seen = 1;
}
void M2006_Get(M2006_t *value)
{
    uint32_t mask = BSP_Lock();
    *value = motor;
    BSP_Unlock(mask);
}
HAL_StatusTypeDef M2006_Send(int16_t current)
{
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;
    uint8_t data[8] = {0};
    unsigned slot = (GIMBAL_DIAL_RX_ID - 0x201U) * 2U;
    data[slot] = (uint8_t)((uint16_t)current >> 8);
    data[slot + 1U] = (uint8_t)current;
    return CAN_SendData(&hfdcan2, GIMBAL_DIAL_COMMAND_ID, data);
}
