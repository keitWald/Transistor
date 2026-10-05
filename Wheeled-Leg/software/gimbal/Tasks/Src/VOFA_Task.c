#include "vofa.h"
#include "UI.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
Vofa_t Vofa;
Vofa_Diagnostics_t vofa_diagnostics;
static char rx_line[64];
static uint8_t rx_used, rx_discard;
static uint32_t period_ms;
_Static_assert(sizeof(float) == 4, "VOFA float32 wire format");
void Vofa_ResetRx(void) { rx_used = rx_discard = 0; }
void Vofa_Init(void)
{
    memset(&Vofa, 0, sizeof(Vofa));
    memset(&vofa_diagnostics, 0, sizeof(vofa_diagnostics));
    period_ms = 20; Vofa_ResetRx();
}
uint32_t Vofa_GetPeriod(void) { return period_ms; }
static void command(void)
{
    if (strcmp(rx_line, "ui=1") == 0) UI_RequestRefresh();
    else if (strncmp(rx_line, "period=", 7) == 0 &&
             rx_line[7] >= '0' && rx_line[7] <= '9') {
        char *end;
        unsigned long value = strtoul(rx_line + 7, &end, 10);
        if (*end != '\0' || value < 10 || value > 1000) {
            vofa_diagnostics.rx_invalid++; return;
        }
        period_ms = (uint32_t)value;
    } else { vofa_diagnostics.rx_invalid++; return; }
    vofa_diagnostics.rx_ok++;
}
void Vofa_PushBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL) return;
    for (uint16_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        if (byte == '\r') continue;
        if (byte == '\n') {
            if (!rx_discard && rx_used != 0) {
                rx_line[rx_used] = '\0'; command();
            }
            Vofa_ResetRx();
        } else if (!rx_discard) {
            if (byte < 32 || byte > 126 || rx_used == sizeof(rx_line) - 1U) {
                rx_discard = 1; vofa_diagnostics.rx_invalid++;
            } else rx_line[rx_used++] = (char)byte;
        }
    }
}
void Vofa_JustFloat(float *data, uint8_t count)
{
    uint8_t output[64];
    uint16_t length = Vofa_PackJustFloat(output, data, count);
    if (length != 0 && HAL_UART_Transmit(&huart7, output, length, 15) != HAL_OK)
        vofa_diagnostics.tx_error++;
}
void Vofa_FireWater(const char *format, ...)
{
    if (format == NULL) return;
    char output[100];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(output, sizeof(output), format, args);
    va_end(args);
    if (length <= 0) return;
    if (length >= (int)sizeof(output)) length = sizeof(output) - 1;
    if (HAL_UART_Transmit(&huart7, (uint8_t *)output, (uint16_t)length, 15) != HAL_OK)
        vofa_diagnostics.tx_error++;
}

#include "VOFA_Task.h"
#include "Gimbal_Control_Task.h"

#include "BSP_Usart.h"
#include "M3508_Motor.h"
#include "J4310_Motor.h"
#include "M2006_Motor.h"
#include "Cloud_Control.h"
#include "FrictionWheel.h"
#include "Shoot.h"
#include "Dial.h"
#include "SBUS.h"
void StartVOFATask(void *argument)
{
    (void)argument;
    Gimbal_WaitReady(); Vofa_Init();
    uint32_t wake = osKernelGetTickCount(), last_tx = HAL_GetTick();
    for (;;) {
        uint8_t bytes[64];
        int32_t count;
        while ((count = BSP_UART_Read(&huart7, bytes, sizeof(bytes))) > 0)
            Vofa_PushBytes(bytes, (uint16_t)count);
        if (count < 0) Vofa_ResetRx();
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_tx) >= Vofa_GetPeriod()) {
            last_tx = now;
            M3508_t friction_motor[2]; J4310_t pitch; M2006_t feeder;
            Cloud_t cloud; Fric_Data_t friction; Shoot_Data_t shoot;
            Dial_Data_t dial; SBUS_RevPack_t remote;
            M3508_Get(friction_motor); J4310_Get(&pitch); M2006_Get(&feeder);
            Cloud_Get(&cloud); Fric_Get(&friction); Shoot_Get(&shoot);
            Dial_Get(&dial); SBUS_Get(&remote);
            Vofa.pitch = pitch.position; /* rad; motor position, not INS attitude. */
            Vofa.data[0] = pitch.position;
            Vofa.data[1] = cloud.Target_Pitch;
            Vofa.data[2] = pitch.velocity;
            Vofa.data[3] = pitch.torque;
            Vofa.data[4] = friction_motor[0].rpm;
            Vofa.data[5] = friction_motor[1].rpm;
            Vofa.data[6] = friction.Required_Speed;
            Vofa.data[7] = feeder.rpm;
            Vofa.data[8] = dial.requested_rpm;
            Vofa.data[9] = shoot.estimated_heat;
            Vofa.data[10] = shoot.Heat_Limit;
            Vofa.data[11] = remote.status_flag;
            Vofa.data[12] = (float)dial.fed_count;
            Vofa_JustFloat(Vofa.data, 13);
        }
        Gimbal_DelayUntil(&wake, 5);
    }
}

