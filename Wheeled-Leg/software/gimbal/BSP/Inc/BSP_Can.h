#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "fdcan.h"
#include "typedef.h"
typedef struct {
    uint32_t tx_busy[3], tx_error[3], rx_rejected[3];
} BSP_CAN_Diagnostics_t;
extern volatile BSP_CAN_Diagnostics_t bsp_can_diagnostics;
HAL_StatusTypeDef CAN_IT_Init(FDCAN_HandleTypeDef *can);
void CAN_AbortPending(FDCAN_HandleTypeDef *can);
HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *can, uint16_t id,
                              const uint8_t data[8]);
#endif

