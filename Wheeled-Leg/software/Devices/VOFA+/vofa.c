#include "vofa.h"
#include "stm32h7xx_hal_uart.h"
#include "cmsis_os2.h"
#include <stdbool.h>
uint8_t vofaTxBuffer[100];
Vofa_t Vofa;

// VOFA UART TX mutex: Vofa_JustFloat can be called from several tasks
// (1 kHz control loop + low-speed debug task). Without serialization one task
// can overwrite vofaTxBuffer while another task's DMA is still reading it, and
// a second HAL_UART_Transmit_DMA may reconfigure the busy DMA stream, wedging
// UART7 TX so data stops updating (RST only recovers it temporarily).
static osMutexId_t vofa_tx_mutex = NULL;
static const osMutexAttr_t vofa_tx_mutex_attr = {
    .name = "VofaTxMutex",
    .attr_bits = osMutexPrioInherit,
    .cb_mem = NULL,
    .cb_size = 0U,
};

static void Vofa_TxLockInit(void) {
  if (vofa_tx_mutex == NULL) {
    vofa_tx_mutex = osMutexNew(&vofa_tx_mutex_attr);
  }
}

static bool Vofa_TxLock(void) {
  Vofa_TxLockInit();
  return (vofa_tx_mutex != NULL) &&
         (osMutexAcquire(vofa_tx_mutex, osWaitForever) == osOK);
}

static void Vofa_TxUnlock(void) {
  if (vofa_tx_mutex != NULL) {
    osMutexRelease(vofa_tx_mutex);
  }
}

// 发送门控与自恢复看门狗状态（供 Vofa_FireWater / Vofa_JustFloat 共用）：
// vofa_tx_busy 在启动 DMA 时置位，由 DMA 完成回调（ISR）或看门狗清除，
// 不依赖 HAL 内部 gState 状态机，避免调试器打断后长期卡死。
static volatile bool vofa_tx_busy = false;
static uint32_t vofa_tx_stall_ticks = 0;

// 按printf格式写，最后必须加\r\n
void Vofa_FireWater(const char *format, ...) {
  static uint8_t txBuffer[100]; // DMA sends asynchronously: buffer must persist
  uint32_t n;
  va_list args;
  va_start(args, format);
  n = vsnprintf((char *)txBuffer, 100, format, args);

  if (Vofa_TxLock()) {
    // VOFA 只需要确认上一帧 DMA 已结束即可：用自管理 busy 标志而非
    // HAL_UART_GetState()（gState|RxState 或值会被遗留的 RX 状态干扰）。
    if (!vofa_tx_busy && huart7.gState == HAL_UART_STATE_READY) {
      if (HAL_UART_Transmit_DMA(&huart7, (uint8_t *)txBuffer, (uint16_t)n) == HAL_OK) {
        vofa_tx_busy = true;
      }
    }
    Vofa_TxUnlock();
  }

  va_end(args);
}

// 输入个数和数组地址s
// TX 自恢复看门狗：若上一帧 DMA 长时间未完成（例如无线调试器打断后丢失完成中断、
// gState 卡在 BUSY_TX），后续帧会被丢弃，表现为 VOFA 数据停止更新、需要按 RST 才能恢复。
// 计数超时后强制复位发送状态（Vofa_UartErrorCallback 直接复位，不依赖异步 DMA 中止）。
void Vofa_JustFloat(float *_data, uint8_t _num) {
  const uint8_t max_float_count =
      (uint8_t)((sizeof(vofaTxBuffer) - 4U) / sizeof(float));
  if (_data == NULL || _num == 0U || _num > max_float_count) {
    return;
  }

  if (Vofa_TxLock()) {
    // The UART7 DMA completion path can leave TX busy indefinitely.  This is
    // called only by the telemetry task, so a bounded synchronous transfer is
    // safe and guarantees a complete JustFloat frame every task period.
    memcpy(vofaTxBuffer, _data, _num * sizeof(float));
    const uint8_t temp_end[4] = {0x00, 0x00, 0x80, 0x7F};
    memcpy(vofaTxBuffer + (_num * sizeof(float)), temp_end, sizeof(temp_end));
    const uint16_t tx_len =
        (uint16_t)((_num * sizeof(float)) + sizeof(temp_end));
    if (HAL_UART_Transmit(&huart7, vofaTxBuffer, tx_len, 10U) != HAL_OK) {
      Vofa_UartErrorCallback(&huart7);
    }
    Vofa_TxUnlock();
  }
  return;

#if 0
  if (Vofa_TxLock()) {
    // 仅当上一帧已发送完成（busy 清除且 gState 回到 READY）才启动新帧，
    // 否则丢弃本帧，避免覆盖仍在 DMA 读取中的 vofaTxBuffer。
    if (!vofa_tx_busy && huart7.gState == HAL_UART_STATE_READY) {
      vofa_tx_stall_ticks = 0;
      memcpy(vofaTxBuffer, _data, _num * 4);
      uint8_t temp_end[4] = {0x00, 0x00, 0x80, 0x7F};
      memcpy(vofaTxBuffer + (_num * 4), temp_end, 4);

      const uint16_t tx_len = (uint16_t)((_num * 4U) + 4U);
      if (HAL_UART_Transmit_DMA(&huart7, vofaTxBuffer, tx_len) == HAL_OK) {
        vofa_tx_busy = true; // 由 TxCplt 回调或看门狗复位
      } else {
        ++vofa_tx_stall_ticks; // HAL 拒绝启动（如 gState 异常），计入卡死计数
      }
    } else if (++vofa_tx_stall_ticks >= 20U) {
      // 单帧 32B @115200 约 2.8ms；发送周期 10ms，连续约 200ms 未完成即视为
      // 发送状态卡死，主动复位
      vofa_tx_stall_ticks = 0;
      vofa_tx_busy = false;
      Vofa_UartErrorCallback(&huart7);
    }
    Vofa_TxUnlock();
  }
}

#endif
}
// DMA TX complete callback, forwarded from HAL_UART_TxCpltCallback (bsp_uart.c)
void Vofa_UartTxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART7) {
    vofa_tx_busy = false; // 允许启动下一帧
  }
}

// TX error recovery: force UART7 TX back to READY so the next frame can
// start. Called from the stall watchdog (task context) and from
// HAL_UART_ErrorCallback (ISR context) - keep it non-blocking.
// Deliberately avoid HAL_UART_AbortTransmit(): its asynchronous DMA abort can
// race with the completing transfer and leave gState stuck at BUSY_TX (VOFA
// then stays dead until a hardware reset). VOFA is the only UART7 TX user and
// the next HAL_UART_Transmit_DMA() reconfigures the DMA stream, so resetting
// the software state directly is safe even if a stale completion arrives later.
void Vofa_UartErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART7) {
    vofa_tx_busy = false;
    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);              // no pending TC interrupt
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);       // stop UART DMA TX request
    huart->gState = HAL_UART_STATE_READY;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_TCF);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
  }
}
