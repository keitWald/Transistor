#ifndef CLOUD_CONTROL_H
#define CLOUD_CONTROL_H
#include "SBUS.h"
typedef struct {
    float Pitch_Raw, Target_Pitch;
    uint8_t active, fault_latched;
} Cloud_t;
void Cloud_Init(void);
void Cloud_Sport_Out(const SBUS_RevPack_t *remote, float dt);
void Cloud_Get(Cloud_t *state);
#endif

