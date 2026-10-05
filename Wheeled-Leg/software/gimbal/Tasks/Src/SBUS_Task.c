#include "SBUS_Task.h"
#include "Gimbal_Control_Task.h"
#include "BSP_Usart.h"
#include "SBUS.h"

void StartSBUSTask(void *argument)
{
    (void)argument;
    Gimbal_WaitReady();
    uint8_t bytes[64];
    uint32_t wake = osKernelGetTickCount();
    for (;;) {
        int32_t count;
        do {
            count = BSP_UART_Read(&huart5, bytes, sizeof(bytes));
            if (count < 0) SBUS_Reset();
            else if (count > 0) SBUS_PushBytes(bytes, (uint16_t)count);
        } while (count > 0);
        Gimbal_DelayUntil(&wake, 1);
    }
}


