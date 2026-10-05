#ifndef GIMBAL_J4310_MOTOR_H
#define GIMBAL_J4310_MOTOR_H
#include "main.h"
typedef struct {
    float position, velocity, torque;
    uint8_t state, mos_temperature, rotor_temperature, seen;
    uint32_t last_ms;
} J4310_t;
void J4310_Receive(uint16_t id, const uint8_t data[8]);
void J4310_Get(J4310_t *motor);
HAL_StatusTypeDef J4310_Special(uint8_t command);
HAL_StatusTypeDef J4310_Send(float position, float velocity, float kp, float kd, float torque);
int J4310_PackMIT(uint8_t data[8], float position, float velocity, float kp, float kd, float torque);
#endif
