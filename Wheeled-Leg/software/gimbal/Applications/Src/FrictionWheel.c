#include "FrictionWheel.h"
#include "Gimbal_Config.h"
#include "M3508_Motor.h"
#include "pid.h"
#include "BSP_Lock.h"
#include "BSP_Can.h"
#include <math.h>
#include <string.h>

static Fric_Data_t state;
static incrementalpid_t pid[2];
static uint32_t ready_since;
static uint8_t ready_tracking;
void Fric_Init(void)
{
    memset(&state, 0, sizeof(state));
    for (unsigned i = 0; i < 2; ++i)
        Incremental_PIDInit(&pid[i], GIMBAL_FRIC_KP, GIMBAL_FRIC_KI, 0,
                            GIMBAL_FRIC_CURRENT_MAX, GIMBAL_FRIC_I_LIMIT);
    ready_tracking = 0;
}
void Fric_Processing(const SBUS_RevPack_t *remote, float dt)
{
    M3508_t motors[2];
    M3508_Get(motors);
    uint32_t now = HAL_GetTick();
    uint8_t online = 1;
    for (unsigned i = 0; i < 2; ++i) {
        if (!motors[i].seen || (uint32_t)(now - motors[i].last_ms) > GIMBAL_MOTOR_TIMEOUT_MS ||
            motors[i].temperature >= GIMBAL_MOTOR_TEMP_LIMIT) online = 0;
    }
    if (!remote->armed) state.fault_latched = 0;
    else if (state.Fric_Switch && !online) state.fault_latched = 1;
    uint8_t permit = (uint8_t)(remote->armed && remote->friction_on && online &&
                               !state.fault_latched);
    if (!permit) {
        if (state.Fric_Switch) CAN_AbortPending(&hfdcan1);
        state.Fric_Switch = state.Fric_Ready = 0;
        state.Required_Speed = 0; ready_tracking = 0;
        Clear_IncrementalPIDData(&pid[0]); 
        Clear_IncrementalPIDData(&pid[1]);
        (void)M3508_Send(0, 0);
        return;
    }
    if (!isfinite(dt) || dt <= 0 || dt > 0.02f) {
        dt = 0.002f;
    }
    state.Fric_Switch = 1;
    state.Required_Speed += GIMBAL_FRIC_RAMP_RPM_S * dt;
    if (state.Required_Speed > GIMBAL_FRIC_RPM) {
        state.Required_Speed = GIMBAL_FRIC_RPM;
    }
    float left_target = state.Required_Speed * GIMBAL_FRIC_LEFT_SIGN;
    float right_target = state.Required_Speed * GIMBAL_FRIC_RIGHT_SIGN;
    int16_t left = (int16_t)Incremental_PID(&pid[0], left_target, motors[0].rpm);
    int16_t right = (int16_t)Incremental_PID(&pid[1], right_target, motors[1].rpm);
    HAL_StatusTypeDef tx = M3508_Send(left, right);
    uint8_t settled = (uint8_t)(tx == HAL_OK && state.Required_Speed == GIMBAL_FRIC_RPM &&
        fabsf(left_target - motors[0].rpm) <= GIMBAL_FRIC_READY_RPM &&
        fabsf(right_target - motors[1].rpm) <= GIMBAL_FRIC_READY_RPM);
    if (!settled) {
        ready_tracking = state.Fric_Ready = 0;
    } else {
        if (!ready_tracking) { ready_since = now; ready_tracking = 1; }
        state.Fric_Ready = (uint8_t)((uint32_t)(now - ready_since) >= GIMBAL_FRIC_READY_MS);
    }
}
void Fric_Get(Fric_Data_t *value)
{
    uint32_t mask = BSP_Lock();
    *value = state;
    BSP_Unlock(mask);
}


