#include "Referee_Task.h"
#include "Gimbal_Control_Task.h"
#include "BSP_Usart.h"
#include "Protocol_Referee.h"
#include "SBUS.h"
#include "UI.h"

void StartRefereeTask(void *argument)
{
    (void)argument;
    Gimbal_WaitReady();
    UI_Init();
    uint8_t bytes[64], refresh_before = 0;
    uint32_t wake = osKernelGetTickCount();
    for (;;) {
        int32_t count;
        do {
            count = BSP_UART_Read(&huart10, bytes, sizeof(bytes));
            if (count < 0) Referee_ResetParser();
            else if (count > 0) Referee_PushBytes(bytes, (uint16_t)count);
        } while (count > 0);
        SBUS_RevPack_t remote;
        SBUS_Get(&remote);
        if (remote.valid && remote.ui_refresh && !refresh_before) UI_RequestRefresh();
        refresh_before = (uint8_t)(remote.valid && remote.ui_refresh);
        UI_Update(HAL_GetTick());
        Gimbal_DelayUntil(&wake, 5);
    }
}


