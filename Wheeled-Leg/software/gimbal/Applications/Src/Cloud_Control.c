#include "Cloud_Control.h"
#include "Gimbal_Config.h"
#include "J4310_Motor.h"
#include "BSP_Lock.h"
#include "BSP_Can.h"
#include <string.h>
#include <math.h>

static Cloud_t cloud;
static uint32_t last_special_ms;
static uint8_t special_sent;
static uint8_t last_special_command;

void Cloud_Init(void)
{
    memset(&cloud, 0, sizeof(cloud));
    special_sent = 0;
    last_special_command = 0;
}
void Cloud_Sport_Out(const SBUS_RevPack_t *remote, float dt)
{
    J4310_t motor;
    J4310_Get(&motor);
    uint32_t now = HAL_GetTick();
    cloud.Pitch_Raw = motor.position;
    if (!remote->armed) cloud.fault_latched = 0;
    if (motor.seen && (motor.state >= 8U ||
        motor.mos_temperature >= GIMBAL_MOTOR_TEMP_LIMIT ||
        motor.rotor_temperature >= GIMBAL_MOTOR_TEMP_LIMIT)) cloud.fault_latched = 1;
    if (cloud.active && (!motor.seen ||
        (uint32_t)(now - motor.last_ms) > GIMBAL_MOTOR_TIMEOUT_MS)) cloud.fault_latched = 1;

    uint8_t permit = (uint8_t)(remote->armed && GIMBAL_PITCH_CALIBRATED &&
                               !cloud.fault_latched);
    if (!permit) {
        uint8_t was_active = cloud.active;
        cloud.active = 0;
        if (was_active || last_special_command != 0xFDU) CAN_AbortPending(&hfdcan2);
        if (was_active || last_special_command != 0xFDU || !special_sent ||
            (uint32_t)(now - last_special_ms) >= 100U) {
            if (J4310_Special(0xFD) == HAL_OK) {
                special_sent = 1; last_special_ms = now; last_special_command = 0xFD;
            }
        }
        return;
    }
    uint8_t online = (uint8_t)(motor.seen &&
        (uint32_t)(now - motor.last_ms) <= GIMBAL_MOTOR_TIMEOUT_MS);
    if (!online || motor.state != 1U) {
        cloud.active = 0;
        if (!special_sent || last_special_command != 0xFCU ||
            (uint32_t)(now - last_special_ms) >= 100U) {
            if (J4310_Send(0, 0, 0, 0, 0) == HAL_OK &&
                J4310_Special(0xFC) == HAL_OK) {
                special_sent = 1; last_special_ms = now; last_special_command = 0xFC;
            }
        }
        return;
    }
    if (!cloud.active) {
        if (motor.position < GIMBAL_PITCH_MIN_RAD || motor.position > GIMBAL_PITCH_MAX_RAD) {
            cloud.fault_latched = 1;
            (void)J4310_Special(0xFD);
            return;
        }
        cloud.Target_Pitch = motor.position;
        cloud.active = 1;
    }
    if (!isfinite(dt) || dt <= 0 || dt > 0.02f) dt = 0.002f;
    cloud.Target_Pitch += remote->pitch_rate * GIMBAL_PITCH_DIRECTION * dt;
    if (cloud.Target_Pitch < GIMBAL_PITCH_MIN_RAD) cloud.Target_Pitch = GIMBAL_PITCH_MIN_RAD;
    if (cloud.Target_Pitch > GIMBAL_PITCH_MAX_RAD) cloud.Target_Pitch = GIMBAL_PITCH_MAX_RAD;
    (void)J4310_Send(cloud.Target_Pitch, 0, GIMBAL_PITCH_KP,
                           GIMBAL_PITCH_KD, GIMBAL_PITCH_FEEDFORWARD);
}
void Cloud_Get(Cloud_t *state)
{
    uint32_t mask = BSP_Lock();
    *state = cloud;
    BSP_Unlock(mask);
}


