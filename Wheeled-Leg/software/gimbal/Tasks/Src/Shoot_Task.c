#include "Shoot_Task.h"
#include "Gimbal_Control_Task.h"
#include "Shoot.h"
#include "main.h"

void StartShootTask(void *argument)
{
    (void)argument;
    Gimbal_WaitReady();
    uint32_t wake = osKernelGetTickCount(), previous = HAL_GetTick();
    for (;;) {
        uint32_t now = HAL_GetTick();
        SBUS_RevPack_t remote;
        SBUS_Get(&remote);
        Shoot_Processing(&remote, (float)(uint32_t)(now - previous) / 1000.0f);
        previous = now;
        Gimbal_DelayUntil(&wake, 2);
    }
}


