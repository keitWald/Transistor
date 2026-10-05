#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H
#include <stdint.h>
/* Existing incremental PID API, reduced to the controller actually used here. */
typedef struct {
    float Kp, Ki, Kd;
    float MaxOutput, IntegralLimit;
    float err, err_last, err_beforeLast;
    float p_out, i_out, d_out, pwm, Target, Measured;
} incrementalpid_t;
float Incremental_PID(incrementalpid_t *pid, float target, float measured);
void Incremental_PIDInit(incrementalpid_t *pid, float kp, float ki, float kd,
                         uint32_t max_output, uint32_t integral_limit);
void Clear_IncrementalPIDData(incrementalpid_t *pid);
#endif

