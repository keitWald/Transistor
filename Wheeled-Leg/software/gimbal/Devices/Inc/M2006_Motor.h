#ifndef GIMBAL_M2006_MOTOR_H
#define GIMBAL_M2006_MOTOR_H
#include "main.h"
typedef struct {
    uint16_t encoder;
    int16_t rpm, current;
    int64_t total_ticks;
    uint32_t last_ms, continuity;
    uint8_t seen;
} M2006_t;
void M2006_Receive(uint16_t id, const uint8_t data[8]);
void M2006_Get(M2006_t *motor);
HAL_StatusTypeDef M2006_Send(int16_t current);
#endif
