#ifndef GIMBAL_SHOOT_H
#define GIMBAL_SHOOT_H
#include "SBUS.h"
typedef struct {
    float estimated_heat;
    uint16_t Heat_Now, Heat_Limit;
    uint8_t Shoot_Switch, heat_permitted;
} Shoot_Data_t;
void Shoot_Init(void);
void Shoot_Processing(const SBUS_RevPack_t *remote, float dt);
void Shoot_Get(Shoot_Data_t *state);
void Shoot_ReportShot(void);
#endif

