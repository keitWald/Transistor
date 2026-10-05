#ifndef FRICTIONWHEEL_H
#define FRICTIONWHEEL_H
#include "SBUS.h"
typedef struct {
    float Required_Speed;
    uint8_t Fric_Ready, Fric_Switch, fault_latched;
} Fric_Data_t;
void Fric_Init(void);
void Fric_Processing(const SBUS_RevPack_t *remote, float dt);
void Fric_Get(Fric_Data_t *state);
#endif

