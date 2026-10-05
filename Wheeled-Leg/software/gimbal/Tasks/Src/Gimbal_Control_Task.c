#include "Gimbal_Control_Task.h"
#include "Gimbal_Config.h"
#include "Cloud_Control.h"
#include "Shoot.h"
#include "Dial.h"
#include "SBUS.h"
#include "Protocol_Referee.h"
#include "BSP_Can.h"
#include "BSP_Usart.h"
#include "BSP_Lock.h"
#include "BSP_BoardCommunication.h"

void Gimbal_WaitReady(void)
{
    if ((osEventFlagsWait(gimbal_ready_event, GIMBAL_READY_FLAG,
                         osFlagsWaitAll | osFlagsNoClear, osWaitForever) & osFlagsError) != 0U)
        Error_Handler();
}
void Gimbal_DelayUntil(uint32_t *wake, uint32_t milliseconds)
{
    uint32_t period = (milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    if (period == 0U) period = 1;
    uint32_t target = *wake + period, now = osKernelGetTickCount();
    if ((int32_t)(target - now) <= 0) target = now + period;
    *wake = target;
    (void)osDelayUntil(target);
}
void Gimbal_Control_Task(void *argument)
{
    (void)argument;
    /* Start interrupts only AFTER the scheduler is running. */
    SBUS_Init(); 
    Referee_Init(); 
    Cloud_Init(); 
    Shoot_Init();
    if (CAN_IT_Init(&hfdcan1) != HAL_OK || CAN_IT_Init(&hfdcan2) != HAL_OK ||
        CAN_IT_Init(&hfdcan3) != HAL_OK || BSP_UART_Init() != HAL_OK)
        Error_Handler();
    if ((osEventFlagsSet(gimbal_ready_event, GIMBAL_READY_FLAG) & osFlagsError) != 0U)
        Error_Handler();

    uint32_t wake = osKernelGetTickCount(), previous = HAL_GetTick(), board_ms = previous;
    uint8_t board_was_armed = 0;
    for (;;) {
        uint32_t now = HAL_GetTick();
        float dt = (float)(uint32_t)(now - previous) / 1000.0f;
        previous = now;
        BSP_UART_Service();
        SBUS_RevPack_t remote;
        SBUS_Get(&remote);
        if (board_was_armed && !remote.armed) 
        {
            CAN_AbortPending(&hfdcan3);
        }
        board_was_armed = remote.armed;
        Cloud_Sport_Out(&remote, dt);

        Shoot_Data_t shoot; Dial_Data_t dial;
        Shoot_Get(&shoot); Dial_Get(&dial);
        uint32_t mask = BSP_Lock();
        ControlMes.x_velocity = (int16_t)(remote.speed / SPEED_MAX * GIMBAL_BOARD_INPUT_SCALE);
        ControlMes.y_velocity = (int16_t)(remote.w_speed / W_SPEED_MAX * GIMBAL_BOARD_INPUT_SCALE);
        ControlMes.z_rotation_velocity = 0;
        ControlMes.yaw_velocity = (int16_t)(remote.yaw_speed_mapped / YAW_SPEED_MAX * GIMBAL_BOARD_INPUT_SCALE);
        ControlMes.pitch_velocity = (int16_t)(remote.pitch_rate * GIMBAL_BOARD_INPUT_SCALE);
        ControlMes.fric_Flag = remote.friction_on;
        ControlMes.shoot_Speed = (shoot.Shoot_Switch && dial.requested_rpm != 0) ? 2U : 0U;
        ControlMes.shoot_state = (uint8_t)(dial.requested_rpm != 0);
        BSP_Unlock(mask);
        if ((uint32_t)(now - board_ms) >= GIMBAL_BOARD_PERIOD_MS) {
            board_ms = now;
            Board1_To_2();
        }
        Gimbal_DelayUntil(&wake, 2);
    }
}



