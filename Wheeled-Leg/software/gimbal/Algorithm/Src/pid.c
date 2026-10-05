#include "pid.h"
#include <math.h>
#include <string.h>

static float limit(float value, float maximum)
{
    if (value > maximum) return maximum;
    if (value < -maximum) return -maximum;
    return value;
}
float Incremental_PID(incrementalpid_t *pid, float target, float measured)
{
    if (!isfinite(target) || !isfinite(measured)) {
        Clear_IncrementalPIDData(pid);
        return 0;
    }
    pid->Target = target;
    pid->Measured = measured;
    pid->err = target - measured;
    pid->p_out = pid->Kp * (pid->err - pid->err_last);
    pid->i_out = limit(pid->Ki * pid->err, pid->IntegralLimit);
    pid->d_out = pid->Kd * (pid->err - 2.0f * pid->err_last + pid->err_beforeLast);
    pid->pwm = limit(pid->pwm + pid->p_out + pid->i_out + pid->d_out, pid->MaxOutput);
    pid->err_beforeLast = pid->err_last;
    pid->err_last = pid->err;
    return pid->pwm;
}
void Incremental_PIDInit(incrementalpid_t *pid, float kp, float ki, float kd,
                         uint32_t max_output, uint32_t integral_limit)
{
    memset(pid, 0, sizeof(*pid));
    pid->Kp = kp; pid->Ki = ki; pid->Kd = kd;
    pid->MaxOutput = (float)max_output;
    pid->IntegralLimit = (float)integral_limit;
}
void Clear_IncrementalPIDData(incrementalpid_t *pid)
{
    pid->err = pid->err_last = pid->err_beforeLast = 0;
    pid->p_out = pid->i_out = pid->d_out = pid->pwm = 0;
    pid->Target = pid->Measured = 0;
}


