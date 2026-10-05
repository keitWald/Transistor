/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "../../Tasks/Inc/Gimbal_Control_Task.h"
#include "../../Tasks/Inc/Referee_Task.h"
#include "../../Tasks/Inc/SBUS_Task.h"
#include "../../Tasks/Inc/Shoot_Task.h"
#include "../../Tasks/Inc/VOFA_Task.h"

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
osEventFlagsId_t gimbal_ready_event;
static osThreadId_t sbus_task_handle;
static osThreadId_t shoot_task_handle;
static osThreadId_t referee_task_handle;
static osThreadId_t vofa_task_handle;
static const osThreadAttr_t vofa_attributes = {
    .name = "VOFA", .stack_size = 1536, .priority = osPriorityBelowNormal};
static const osThreadAttr_t sbus_attributes = {
    .name = "SBUS", .stack_size = 1024, .priority = osPriorityHigh};
static const osThreadAttr_t shoot_attributes = {
    .name = "Shoot", .stack_size = 1536, .priority = osPriorityAboveNormal};
static const osThreadAttr_t referee_attributes = {
    .name = "Referee", .stack_size = 2048, .priority = osPriorityNormal};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
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
void StartControlLoopTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  gimbal_ready_event = osEventFlagsNew(NULL);
  if (gimbal_ready_event == NULL)
    Error_Handler();
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
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  ControlLoopTaskHandle =
      osThreadNew(StartControlLoopTask, NULL, &ControlLoopTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  sbus_task_handle = osThreadNew(StartSBUSTask, NULL, &sbus_attributes);
  shoot_task_handle = osThreadNew(StartShootTask, NULL, &shoot_attributes);
  referee_task_handle =
      osThreadNew(StartRefereeTask, NULL, &referee_attributes);
  vofa_task_handle = osThreadNew(StartVOFATask, NULL, &vofa_attributes);
  if (defaultTaskHandle == NULL || ControlLoopTaskHandle == NULL ||
      sbus_task_handle == NULL || shoot_task_handle == NULL ||
      referee_task_handle == NULL || vofa_task_handle == NULL)
    Error_Handler();
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
  (void)argument;
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
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
  Gimbal_Control_Task(argument);
  /* USER CODE END StartControlLoopTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

