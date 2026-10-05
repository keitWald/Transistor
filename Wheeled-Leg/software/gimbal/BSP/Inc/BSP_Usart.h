#ifndef BSP_USART_H
#define BSP_USART_H
#include "usart.h"
typedef struct { uint32_t overflow, errors, restart_failed; } BSP_UART_Diagnostics_t;
extern volatile BSP_UART_Diagnostics_t bsp_uart_diagnostics[3];
HAL_StatusTypeDef BSP_UART_Init(void);
void BSP_UART_Service(void);
/* Nonblocking; -1 means input was lost, caller must reset its parser. */
int32_t BSP_UART_Read(UART_HandleTypeDef *uart, uint8_t *data, uint16_t capacity);
HAL_StatusTypeDef BSP_Referee_Transmit(uint8_t *data, uint16_t length);
#endif


