#ifndef GIMBAL_M3508_MOTOR_H
#define GIMBAL_M3508_MOTOR_H
#include "main.h"
typedef struct {
    uint16_t encoder;
    int16_t rpm, current;
    uint8_t temperature, seen;
    uint32_t last_ms;
} M3508_t;
void M3508_Receive(uint16_t id, const uint8_t data[8]);
void M3508_Get(M3508_t motors[2]);
HAL_StatusTypeDef M3508_Send(int16_t left, int16_t right);
#endif
