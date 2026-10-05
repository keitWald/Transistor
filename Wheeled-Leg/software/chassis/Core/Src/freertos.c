/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "N100.h"
#include "SBUS.h"
#include "Saber_C3.h"
#include "Task_manager.h"
#include "board_comm.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "vofa.h"
#include "ws2812.h"

// #include "BMI088driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for BoardcommTask */
osThreadId_t BoardcommTaskHandle;
const osThreadAttr_t BoardcommTask_attributes = {
    .name = "BoardcommTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for VofaTask */
osThreadId_t VofaTaskHandle;
const osThreadAttr_t VofaTask_attributes = {
    .name = "VofaTask",
    .stack_size = 128 * 4,
    // Keep telemetry below the 1 kHz realtime control loop, but above ordinary
    // background work.  At Low priority it was starved whenever the control
    // loop overran, leaving VOFA with only occasional (blue-light) frames.
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for ET16STask */
osThreadId_t ET16STaskHandle;
const osThreadAttr_t ET16STask_attributes = {
    .name = "ET16STask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for CloudTask */
osThreadId_t CloudTaskHandle;
const osThreadAttr_t CloudTask_attributes = {
    .name = "CloudTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for ControlLoopTask */
osThreadId_t ControlLoopTaskHandle;
const osThreadAttr_t ControlLoopTask_attributes = {
    .name = "ControlLoopTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartBoardcommTask(void *argument);
void StartVofaTask(void *argument);
void StartET16STask(void *argument);
void StartCloudTask(void *argument);
void StartControlLoopTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of BoardcommTask */
  BoardcommTaskHandle =
      osThreadNew(StartBoardcommTask, NULL, &BoardcommTask_attributes);

  /* creation of VofaTask */
  VofaTaskHandle = osThreadNew(StartVofaTask, NULL, &VofaTask_attributes);

  /* creation of ET16STask */
  ET16STaskHandle = osThreadNew(StartET16STask, NULL, &ET16STask_attributes);

  /* creation of CloudTask */
  CloudTaskHandle = osThreadNew(StartCloudTask, NULL, &CloudTask_attributes);

  /* creation of ControlLoopTask */
  ControlLoopTaskHandle =
      osThreadNew(StartControlLoopTask, NULL, &ControlLoopTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    WS2812_Ctrl(50, 0, 0);
    osDelay(500);
    WS2812_Ctrl(10, 10, 50);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartBoardcommTask */
/**
 * @brief Function implementing the BoardcommTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBoardcommTask */
void StartBoardcommTask(void *argument) {
  /* USER CODE BEGIN StartBoardcommTask */
  static float board_start;
  static float board_dt;
  //	 BoardCommInit(&board_comm,&hfdcan2, 0x11f);
  Board2_FUN.BoardCommInit(&hfdcan2);
  /* Infinite loop */
  for (;;) {
    board_start = DWT_GetTimeline_ms();
    boardCommunicateTask();
    board_dt = DWT_GetTimeline_ms() - board_start;
    osDelay(1);
  }
  /* USER CODE END StartBoardcommTask */
}

/* USER CODE BEGIN Header_StartVofaTask */
/**
 * @brief Function implementing the VofaTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartVofaTask */
void StartVofaTask(void *argument) {
  /* USER CODE BEGIN StartVofaTask */
  /* Infinite loop */
  for (;;) {
    vofaTask();
    // 独立于控制环发送：即使控制环异常（卡死/任务未创建），VOFA 仍持续出数据，
    // 便于上位机诊断。13 路 JustFloat 在 115200 baud 下约 4.9 ms，
    // 再延时 5 ms 后保持约 110 Hz，且该发送仍独立于 1 kHz 控制任务。
    Vofa_JustFloat(Vofa.data, 13);
    osDelay(5);
  }
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartET16STask */
/**
 * @brief Function implementing the ET16STask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartET16STask */
void StartET16STask(void *argument) {
  /* USER CODE BEGIN StartET16STask */
  static float ET16S_start;
  static float ET16S_dt;
  SBUS_Init(&huart5);
  /* Infinite loop */
  for (;;) {
    ET16S_start = DWT_GetTimeline_ms();
    SBUS_Handle();
    ET16S_dt = DWT_GetTimeline_ms() - ET16S_start;
    osDelay(1);
  }
  /* USER CODE END StartET16STask */
}

/* USER CODE BEGIN Header_StartCloudTask */
/**
 * @brief Function implementing the CloudTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCloudTask */
void StartCloudTask(void *argument) {
  /* USER CODE BEGIN StartCloudTask */
  static float cloud_start;
  static float cloud_dt;
  CloudInit();
  /* Infinite loop */
  for (;;) {
    cloud_start = DWT_GetTimeline_ms();
    Cloud_ControlTask();
    cloud_dt = DWT_GetTimeline_ms() - cloud_start;
    osDelay(1);
  }
  /* USER CODE END StartCloudTask */
}

/* USER CODE BEGIN Header_StartControlLoopTask */
/**
 * @brief Function implementing the ControlLoopTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlLoopTask */
void StartControlLoopTask(void *argument) {
  /* USER CODE BEGIN StartControlLoopTask */
  // 初始化

  INS_Init();
  ObserverInit();
  ChassisInit();
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for (;;) {
    uint32_t start_time = DWT_GetTimeline_ms();

    // 1. 读传感器 (最先做，保证数据最新)
    INS_Task(); // 读取IMU

    // 2. 状态观测 (紧接传感器)
    ObserveTask(); // 计算卡尔曼滤波/状态观测器

    // 3. 底盘任务
    ChassisTask();

    // 4. 控制指令下发电机
    DM_MotorTask();   // 打包电机CAN协议
    WheelMotorTask(); // 发送CAN数据

    SDM02Task(); // 仅在起跳前压缩准备阶段开启测距

    // 注意：这里不再调用 vofaTask()。vofaTask() 写 Vofa.data，若同时被本 1kHz
    // 控制环与 110Hz 的 VofaTask 任务调用，两个任务会竞争写同一数组，导致
    // data[10](骤降) 与 data[12](置信度) 出现不同步（VOFA 上 conf 偶尔不等于
    // 0.5*pitch+0.5*spike）。VOFA 数据刷新与发送统一由独立任务 StartVofaTask 完成。

    // 性能监测
    float execution_time = DWT_GetTimeline_ms() - start_time;

    // 绝对延时，确保周期精准
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartControlLoopTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
