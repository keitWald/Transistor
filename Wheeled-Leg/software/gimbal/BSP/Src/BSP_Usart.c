#include "BSP_Usart.h"
#include "BSP_Lock.h"
#include <string.h>

#define RX_RING_SIZE 512U
typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t byte, data[RX_RING_SIZE];
    volatile uint16_t head, tail;
    volatile uint8_t lost, restart;
} UART_Ring_t;
static UART_Ring_t rings[3];
volatile BSP_UART_Diagnostics_t bsp_uart_diagnostics[3];

static int index_of(UART_HandleTypeDef *uart)
{
    if (uart == &huart5) return 0;
    if (uart == &huart10) return 1;
    if (uart == &huart7) return 2;
    return -1;
}
HAL_StatusTypeDef BSP_UART_Init(void)
{
    memset(rings, 0, sizeof(rings));
    rings[0].uart = &huart5; rings[1].uart = &huart10; rings[2].uart = &huart7;
    if (huart5.Instance != UART5 || huart10.Instance != USART10 || huart7.Instance != UART7) return HAL_ERROR;
    /* ponytail: one IRQ per byte; switch to DMA in D2 SRAM if profiling requires it.
       CPU-owned buffers work with the existing linker script and D-cache settings. */
    if (HAL_UART_Receive_IT(&huart5, &rings[0].byte, 1) != HAL_OK ||
        HAL_UART_Receive_IT(&huart10, &rings[1].byte, 1) != HAL_OK ||
        HAL_UART_Receive_IT(&huart7, &rings[2].byte, 1) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    int index = index_of(uart);
    if (index < 0) return;
    UART_Ring_t *ring = &rings[index];
    uint16_t next = (uint16_t)((ring->head + 1U) % RX_RING_SIZE);
    if (!ring->lost) {
        if (next == ring->tail) {
            ring->lost = 1;
            bsp_uart_diagnostics[index].overflow++;
        } else {
            ring->data[ring->head] = ring->byte;
            ring->head = next;
        }
    }
    if (HAL_UART_Receive_IT(uart, &ring->byte, 1) != HAL_OK) {
        ring->restart = ring->lost = 1;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    int index = index_of(uart);
    if (index < 0) return;
    bsp_uart_diagnostics[index].errors++;
    rings[index].lost = rings[index].restart = 1;
}
void  BSP_UART_Service(void)
{
    for (int i = 0; i < 3; ++i) {
        UART_Ring_t *ring = &rings[i];
        if (ring->uart == NULL || !ring->restart) continue;
        uint32_t mask = BSP_Lock();
        (void)HAL_UART_AbortReceive(ring->uart);
        if (HAL_UART_Receive_IT(ring->uart, &ring->byte, 1) == HAL_OK) ring->restart = 0;
        else bsp_uart_diagnostics[i].restart_failed++;
        BSP_Unlock(mask);
    }
}
int32_t BSP_UART_Read(UART_HandleTypeDef *uart, uint8_t *data, uint16_t capacity)
{
    int index = index_of(uart);
    if (index < 0 || data == NULL) return -1;
    UART_Ring_t *ring = &rings[index];
    uint32_t mask = BSP_Lock();
    if (ring->lost) {
        ring->tail = ring->head;
        ring->lost = 0;
        BSP_Unlock(mask);
        return -1;
    }
    uint16_t count = 0;
    while (count < capacity && ring->tail != ring->head) {
        data[count++] = ring->data[ring->tail];
        ring->tail = (uint16_t)((ring->tail + 1U) % RX_RING_SIZE);
    }
    BSP_Unlock(mask);
    return count;
}
HAL_StatusTypeDef BSP_Referee_Transmit(uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > 128U) return HAL_ERROR;
    /* Blocking TX in referee task: buffer remains valid until transmission ends. */
    return HAL_UART_Transmit(&huart10, data, length, 20U);
}



