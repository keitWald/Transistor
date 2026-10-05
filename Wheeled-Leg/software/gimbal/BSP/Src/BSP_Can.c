#include "BSP_Can.h"
#include "BSP_Lock.h"
#include "BSP_BoardCommunication.h"
#include "M3508_Motor.h"
#include "J4310_Motor.h"
#include "M2006_Motor.h"

volatile BSP_CAN_Diagnostics_t bsp_can_diagnostics;

static int bus_index(FDCAN_HandleTypeDef *can)
{
    if (can == &hfdcan1) return 0;
    if (can == &hfdcan2) return 1;
    if (can == &hfdcan3) return 2;
    return -1;
}

HAL_StatusTypeDef CAN_IT_Init(FDCAN_HandleTypeDef *can)
{
    FDCAN_FilterTypeDef filter = {0};
    if (bus_index(can) < 0) return HAL_ERROR;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;
    if (HAL_FDCAN_ConfigFilter(can, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(can, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return HAL_ERROR;
    return HAL_FDCAN_Start(can);
}

void CAN_AbortPending(FDCAN_HandleTypeDef *can)
{
    if (bus_index(can) < 0) return;
    uint32_t count = can->Init.TxBuffersNbr + can->Init.TxFifoQueueElmtsNbr;
    if (count == 0U) return;
    uint32_t mask = BSP_Lock();
    (void)HAL_FDCAN_AbortTxRequest(can, count >= 32U ? UINT32_MAX : (1UL << count) - 1UL);
    BSP_Unlock(mask);
}

HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef *can, uint16_t id,
                              const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef result;
    int index = bus_index(can);
    if (index < 0 || id > 0x7FFU || data == NULL) return HAL_ERROR;
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    /* No software command backlog: do not replay stale setpoints. */
    uint32_t mask = BSP_Lock();
    if (HAL_FDCAN_GetTxFifoFreeLevel(can) == 0U) {
        bsp_can_diagnostics.tx_busy[index]++;
        result = HAL_BUSY;
    } else {
        result = HAL_FDCAN_AddMessageToTxFifoQ(can, &header, (uint8_t *)data);
        if (result != HAL_OK) bsp_can_diagnostics.tx_error[index]++;
    }
    BSP_Unlock(mask);
    return result;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *can, uint32_t flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64]; /* HAL copies received length before we validate it. */
    int index = bus_index(can);
    if (index < 0 || (flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(can, FDCAN_RX_FIFO0) != 0U) {
        if (HAL_FDCAN_GetRxMessage(can, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN || header.DataLength != FDCAN_DLC_BYTES_8) {
            bsp_can_diagnostics.rx_rejected[index]++;
            continue;
        }
        if (can == &hfdcan1) M3508_Receive((uint16_t)header.Identifier, data);
        else if (can == &hfdcan2) {
            J4310_Receive((uint16_t)header.Identifier, data);
            M2006_Receive((uint16_t)header.Identifier, data);
        } else Board1_Receive((uint16_t)header.Identifier, data);
    }
}


