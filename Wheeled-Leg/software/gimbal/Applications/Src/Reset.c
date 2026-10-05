/**
 * @file Reset.c
 * @author SJW
 * @brief 复位函数
 * @version 0.1
 * @date 2026-3-10
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "Reset.h"

void System_Reset(void)
{
    if (ControlMes.reset_Flag == 1)
    {
        HAL_NVIC_SystemReset();
    }
}
