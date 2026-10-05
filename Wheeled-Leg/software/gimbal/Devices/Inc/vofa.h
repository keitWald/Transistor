#ifndef GIMBAL_VOFA_H
#define GIMBAL_VOFA_H
#include "usart.h"
/* Same type as chassis/Devices/inc/vofa.h. Data channels are IEEE754 float32. */
typedef struct {
    float roll;
    float yaw;
    float pitch;
    float data[13];
} Vofa_t;
extern Vofa_t Vofa;
typedef struct { uint32_t rx_ok, rx_invalid, tx_error; } Vofa_Diagnostics_t;
extern Vofa_Diagnostics_t vofa_diagnostics;
void Vofa_Init(void);
void Vofa_ResetRx(void);
void Vofa_PushBytes(const uint8_t *data, uint16_t length);
uint32_t Vofa_GetPeriod(void);
/* Only the VOFA task may transmit, so no shared DMA buffer or TX mutex is needed. */
void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *data, uint8_t count);
uint16_t Vofa_PackJustFloat(uint8_t output[64], const float *data, uint8_t count);
#endif
