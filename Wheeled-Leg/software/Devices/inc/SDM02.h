#ifndef __SDM02_H
#define __SDM02_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

/* SDM02 factory UART setting from the product manual: 115200, 8-N-1. */
#define SDM02_UART_BAUD_RATE 115200U
#define SDM02_FRAME_HEADER 0x5CU
#define SDM02_FRAME_LENGTH 4U
#define SDM02_MAX_DISTANCE_MM 2000U
#define SDM02_DATA_FRESH_TIMEOUT_MS 100U

typedef struct {
  volatile uint16_t distance_mm;
  volatile uint8_t data_valid;
  volatile uint8_t measurement_enabled;
  volatile uint32_t frame_count;
  volatile uint32_t checksum_error_count;
  volatile uint32_t uart_error_count;
  volatile uint32_t command_error_count;
  volatile uint32_t last_update_ms;
} SDM02_Data_t;

extern SDM02_Data_t SDM02_Data;

/** Initialize USART10 reception and force the sensor into stopped state. */
HAL_StatusTypeDef SDM02_Init(void);

/**
 * Periodic driver service. Pass 1 only while pre-jump preparation is active.
 * The function is edge-triggered: it sends one start/stop command per change.
 */
void SDM02_Update(uint8_t enable_measurement);

/** Direct start/stop control for diagnostics or alternate applications. */
HAL_StatusTypeDef SDM02_SetMeasurementEnabled(uint8_t enable);

uint16_t SDM02_GetDistanceMm(void);
uint8_t SDM02_IsMeasurementEnabled(void);
uint8_t SDM02_IsDataFresh(void);
uint32_t SDM02_GetTotalErrorCount(void);

/* Forwarded by the central HAL UART callbacks in bsp_uart.c. */
void SDM02_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void SDM02_UartErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __SDM02_H */
