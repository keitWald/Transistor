#include "Dial.h"
#include "Shoot.h"
#include "SBUS.h"
#include "Gimbal_Config.h"
#include "M2006_Motor.h"
#include "pid.h"
#include "BSP_Lock.h"
#include "BSP_Can.h"
#include <math.h>
#include <string.h>
static Dial_Data_t state;
static incrementalpid_t speed_pid;
static int64_t previous_ticks;
static float travel_ticks;
static uint32_t continuity;
static uint32_t stall_since;
static uint8_t stall_tracking;
_Static_assert(GIMBAL_DIAL_POCKETS > 0, "Feeder pocket count must be positive");
void Dial_Init(void)
{
    /* C11 static assertions require integer constant expressions. */
    if (!isfinite(GIMBAL_DIAL_GEAR_RATIO) || GIMBAL_DIAL_GEAR_RATIO <= 0.0f)
        Error_Handler();
    memset(&state, 0, sizeof(state));
    Incremental_PIDInit(&speed_pid, GIMBAL_DIAL_KP, GIMBAL_DIAL_KI, 0,
                        GIMBAL_DIAL_CURRENT_MAX, GIMBAL_DIAL_I_LIMIT);
    previous_ticks = 0; travel_ticks = 0; continuity = 0;
    stall_tracking = 0;
}
void Dial_Processing(uint8_t enabled, float rpm)
{
    M2006_t motor;
    SBUS_RevPack_t remote;
    M2006_Get(&motor); SBUS_Get(&remote);
    uint8_t was_running = (uint8_t)(state.requested_rpm != 0);
    state.installed = GIMBAL_DIAL_INSTALLED;
    state.online = (uint8_t)(state.installed && motor.seen &&
        (uint32_t)(HAL_GetTick() - motor.last_ms) <= GIMBAL_MOTOR_TIMEOUT_MS);
    if (!remote.armed) state.fault_latched = 0;
    else if (was_running && !state.online) state.fault_latched = 1;
    if (was_running && fabsf((float)motor.rpm) < GIMBAL_DIAL_STALL_RPM) {
        if (!stall_tracking) { stall_tracking = 1; stall_since = HAL_GetTick(); }
        if ((uint32_t)(HAL_GetTick() - stall_since) >= GIMBAL_DIAL_STALL_MS)
            state.fault_latched = 1;
    } else stall_tracking = 0;
    if (state.online && continuity == motor.continuity) {
        float delta = (float)(motor.total_ticks - previous_ticks) * GIMBAL_DIAL_DIRECTION;
        if (was_running && delta > 0) travel_ticks += delta;
    } else travel_ticks = 0;
    continuity = motor.continuity; previous_ticks = motor.total_ticks;
    const float ticks_per_shot = 8192.0f * GIMBAL_DIAL_GEAR_RATIO / GIMBAL_DIAL_POCKETS;
    /* ponytail: encoder travel estimates fed bullets; add a projectile sensor
       when distinguishing empty pockets/misfeeds is required. */
    while (travel_ticks >= ticks_per_shot) {
        travel_ticks -= ticks_per_shot;
        state.fed_count++; Dial_ReportShot();
    }
    state.requested_rpm = (enabled && remote.armed && state.online &&
        GIMBAL_DIAL_CALIBRATED && !state.fault_latched && isfinite(rpm)) ?
        rpm * GIMBAL_DIAL_DIRECTION : 0;
    int16_t current = 0;
    if (was_running && state.requested_rpm == 0) CAN_AbortPending(&hfdcan2);
    if (state.requested_rpm == 0) Clear_IncrementalPIDData(&speed_pid);
    else current = (int16_t)Incremental_PID(&speed_pid, state.requested_rpm, motor.rpm);
    if (M2006_Send(current) != HAL_OK) {
        CAN_AbortPending(&hfdcan2);
        state.requested_rpm = 0;
        Clear_IncrementalPIDData(&speed_pid);
        (void)M2006_Send(0);
    }
}
void Dial_Get(Dial_Data_t *value)
{
    uint32_t mask = BSP_Lock();
    *value = state;
    BSP_Unlock(mask);
}
void Dial_ReportShot(void) { Shoot_ReportShot(); }
