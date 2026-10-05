#ifndef GIMBAL_DIAL_H
#define GIMBAL_DIAL_H
#include "main.h"
typedef struct {
    float requested_rpm;
    uint32_t fed_count;
    uint8_t installed, online, fault_latched;
} Dial_Data_t;
void Dial_Init(void);
void Dial_Processing(uint8_t enabled, float rpm);
void Dial_Get(Dial_Data_t *state);
/* Feeder implementation calls this ONCE per physically fed projectile. */
void Dial_ReportShot(void);
#endif

