#ifndef _VOFA_H
#define _VOFA_H

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include "stdint.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  float roll;
  float yaw;
  float pitch;
  float data[10];
} Vofa_t;

#ifdef __cplusplus
extern "C" {
#endif

void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);
void Vofa_UartTxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_UartErrorCallback(UART_HandleTypeDef *huart);
extern Vofa_t Vofa;

#ifdef __cplusplus
}
#endif
#endif
