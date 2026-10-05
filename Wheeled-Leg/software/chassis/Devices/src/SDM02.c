#include "SDM02.h"
#include <string.h>

#define SDM02_RX_DMA_BUFFER_SIZE 32U
#define SDM02_RETRY_INTERVAL_MS 20U
#define SDM02_COMMAND_TIMEOUT_MS 5U

/*
 * The checksum excludes the 0x5A command header. It is the low eight bits of
 * the one's complement of all following bytes before the checksum byte.
 */
static const uint8_t sdm02_start_command[6] = {0x5A, 0x0A, 0x02,
                                               0x02, 0x00, 0xF1};
static const uint8_t sdm02_stop_command[6] = {0x5A, 0x0A, 0x02,
                                              0x00, 0x00, 0xF3};

SDM02_Data_t SDM02_Data;

static uint8_t sdm02_rx_dma_buffer[SDM02_RX_DMA_BUFFER_SIZE];
static uint8_t sdm02_frame[SDM02_FRAME_LENGTH];
static uint8_t sdm02_frame_index;
static uint8_t sdm02_initialized;
static uint8_t sdm02_rx_armed;
static uint8_t sdm02_command_state_known;
static uint32_t sdm02_last_rx_retry_ms;
static uint32_t sdm02_last_command_retry_ms;

static uint8_t SDM02_Checksum(const uint8_t *data, uint16_t length) {
  uint8_t sum = 0U;
  uint16_t i;

  for (i = 0U; i < length; ++i) {
    sum = (uint8_t)(sum + data[i]);
  }
  return (uint8_t)(~sum);
}

static HAL_StatusTypeDef SDM02_StartReceive(void) {
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(
      &huart10, sdm02_rx_dma_buffer, sizeof(sdm02_rx_dma_buffer));

  if (status == HAL_OK) {
    sdm02_rx_armed = 1U;
    if (huart10.hdmarx != NULL) {
      /* Each SDM02 frame is separated by UART idle; a half-DMA event is noise. */
      __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }
  } else {
    sdm02_rx_armed = 0U;
  }
  return status;
}

static void SDM02_ConsumeByte(uint8_t byte) {
  if (sdm02_frame_index == 0U) {
    if (byte == SDM02_FRAME_HEADER) {
      sdm02_frame[0] = byte;
      sdm02_frame_index = 1U;
    }
    return;
  }

  sdm02_frame[sdm02_frame_index++] = byte;
  if (sdm02_frame_index < SDM02_FRAME_LENGTH) {
    return;
  }

  if (SDM02_Checksum(&sdm02_frame[1], 2U) == sdm02_frame[3]) {
    const uint16_t distance_mm =
        (uint16_t)sdm02_frame[1] | ((uint16_t)sdm02_frame[2] << 8U);

    /* Preserve the raw returned value for VOFA, even if it is out of range. */
    SDM02_Data.distance_mm = distance_mm;
    SDM02_Data.data_valid =
        (distance_mm <= SDM02_MAX_DISTANCE_MM) ? 1U : 0U;
    SDM02_Data.last_update_ms = HAL_GetTick();
    ++SDM02_Data.frame_count;
  } else {
    ++SDM02_Data.checksum_error_count;
  }

  /* If the last byte is already the next header, keep it for fast resync. */
  if (byte == SDM02_FRAME_HEADER) {
    sdm02_frame[0] = byte;
    sdm02_frame_index = 1U;
  } else {
    sdm02_frame_index = 0U;
  }
}

HAL_StatusTypeDef SDM02_Init(void) {
  HAL_StatusTypeDef rx_status;
  HAL_StatusTypeDef tx_status;

  memset(&SDM02_Data, 0, sizeof(SDM02_Data));
  memset(sdm02_rx_dma_buffer, 0, sizeof(sdm02_rx_dma_buffer));
  memset(sdm02_frame, 0, sizeof(sdm02_frame));
  sdm02_frame_index = 0U;
  sdm02_rx_armed = 0U;
  sdm02_command_state_known = 0U;
  sdm02_last_rx_retry_ms = 0U;
  sdm02_last_command_retry_ms = 0U;
  sdm02_initialized = 1U;

  rx_status = SDM02_StartReceive();

  /* Stop at boot so the laser is not active outside pre-jump preparation. */
  tx_status = HAL_UART_Transmit(&huart10, (uint8_t *)sdm02_stop_command,
                               sizeof(sdm02_stop_command),
                               SDM02_COMMAND_TIMEOUT_MS);
  if (tx_status != HAL_OK) {
    ++SDM02_Data.command_error_count;
  } else {
    sdm02_command_state_known = 1U;
  }

  if (rx_status != HAL_OK) {
    ++SDM02_Data.uart_error_count;
    return rx_status;
  }
  return tx_status;
}

HAL_StatusTypeDef SDM02_SetMeasurementEnabled(uint8_t enable) {
  const uint8_t requested_state = (enable != 0U) ? 1U : 0U;
  const uint8_t *command;
  HAL_StatusTypeDef status;

  if (sdm02_initialized == 0U) {
    return HAL_ERROR;
  }
  if (sdm02_command_state_known != 0U &&
      requested_state == SDM02_Data.measurement_enabled) {
    return HAL_OK;
  }

  command = requested_state ? sdm02_start_command : sdm02_stop_command;
  status = HAL_UART_Transmit_IT(&huart10, (uint8_t *)command, 6U);
  if (status == HAL_OK) {
    SDM02_Data.measurement_enabled = requested_state;
    sdm02_command_state_known = 1U;
    if (requested_state == 0U) {
      SDM02_Data.data_valid = 0U;
    }
  } else if (status != HAL_BUSY) {
    ++SDM02_Data.command_error_count;
  }
  return status;
}

void SDM02_Update(uint8_t enable_measurement) {
  const uint32_t now = HAL_GetTick();

  if (sdm02_initialized == 0U) {
    return;
  }

  if (sdm02_rx_armed == 0U &&
      (uint32_t)(now - sdm02_last_rx_retry_ms) >= SDM02_RETRY_INTERVAL_MS) {
    sdm02_last_rx_retry_ms = now;
    if (SDM02_StartReceive() != HAL_OK) {
      ++SDM02_Data.uart_error_count;
    }
  }

  if ((sdm02_command_state_known == 0U ||
       ((enable_measurement != 0U) ? 1U : 0U) !=
           SDM02_Data.measurement_enabled) &&
      (uint32_t)(now - sdm02_last_command_retry_ms) >=
          SDM02_RETRY_INTERVAL_MS) {
    sdm02_last_command_retry_ms = now;
    (void)SDM02_SetMeasurementEnabled(enable_measurement);
  }
}

uint16_t SDM02_GetDistanceMm(void) { return SDM02_Data.distance_mm; }

uint8_t SDM02_IsMeasurementEnabled(void) {
  return SDM02_Data.measurement_enabled;
}

uint8_t SDM02_IsDataFresh(void) {
  const uint32_t age = HAL_GetTick() - SDM02_Data.last_update_ms;
  return (SDM02_Data.measurement_enabled != 0U &&
          SDM02_Data.data_valid != 0U &&
          age <= SDM02_DATA_FRESH_TIMEOUT_MS)
             ? 1U
             : 0U;
}

uint32_t SDM02_GetTotalErrorCount(void) {
  return SDM02_Data.checksum_error_count + SDM02_Data.uart_error_count +
         SDM02_Data.command_error_count;
}

void SDM02_UartRxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  uint16_t i;

  if (huart == NULL || huart->Instance != USART10 ||
      sdm02_initialized == 0U) {
    return;
  }

  sdm02_rx_armed = 0U;
  if (size > sizeof(sdm02_rx_dma_buffer)) {
    size = sizeof(sdm02_rx_dma_buffer);
  }
  for (i = 0U; i < size; ++i) {
    SDM02_ConsumeByte(sdm02_rx_dma_buffer[i]);
  }

  if (SDM02_StartReceive() != HAL_OK) {
    ++SDM02_Data.uart_error_count;
  }
}

void SDM02_UartErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == NULL || huart->Instance != USART10) {
    return;
  }

  ++SDM02_Data.uart_error_count;
  SDM02_Data.data_valid = 0U;
  sdm02_rx_armed = 0U;
  sdm02_command_state_known = 0U;
  sdm02_frame_index = 0U;

  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)HAL_UART_AbortReceive(huart);
  if (SDM02_StartReceive() != HAL_OK) {
    ++SDM02_Data.uart_error_count;
  }
}
