#include "Protocol_Referee.h"
#include "Protocol_CRC.h"
#include "BSP_Usart.h"
#include "BSP_Lock.h"
#include "Gimbal_Config.h"
#include <math.h>
#include <string.h>

#define REF_FRAME_CAPACITY 256U
static uint8_t frame[REF_FRAME_CAPACITY], sequence;
static uint16_t used;
static Referee_Data_t referee;
volatile Referee_Diagnostics_t referee_diagnostics;

static uint16_t read16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8);
}
static void write16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8);
}
void Referee_Init(void)
{
    memset(&referee, 0, sizeof(referee));
    memset((void *)&referee_diagnostics, 0, sizeof(referee_diagnostics));
    sequence = 0; used = 0;
}
void Referee_ResetParser(void)
{
    uint32_t mask = BSP_Lock();
    used = 0;
    referee.status_seen = referee.heat_seen = referee.frame_seen = 0;
    BSP_Unlock(mask);
}
static void discard(uint16_t count)
{
    used = (uint16_t)(used - count);
    memmove(frame, frame + count, used);
}
static void solve(uint16_t cmd, const uint8_t *p, uint16_t size)
{
    uint32_t now = HAL_GetTick();
    uint32_t mask = BSP_Lock();
    referee.frame_seen = 1; referee.frame_ms = now;
    switch (cmd) {
    case 0x0001:
        if (size != 11U) break;
        referee.game_progress = p[0] >> 4;
        referee.stage_remaining_seconds = read16(p + 1);
        BSP_Unlock(mask);
        return;
    case 0x0201:
        if (size != 13U) break;
        referee.robot_id = p[0]; referee.robot_level = p[1];
        referee.hp = read16(p + 2); referee.hp_max = read16(p + 4);
        referee.cooling_per_second = read16(p + 6); referee.heat_limit = read16(p + 8);
        referee.chassis_power_limit = read16(p + 10); referee.power_outputs = p[12];
        referee.status_ms = now; referee.status_seen = 1; referee.status_sequence++;
        BSP_Unlock(mask);
        return;
    case 0x0202:
        if (size != 16U) break;
        referee.voltage_mv = read16(p); referee.current_ma = read16(p + 2);
        memcpy(&referee.chassis_power, p + 4, sizeof(float));
        if (!isfinite(referee.chassis_power)) { referee.heat_seen = 0; break; }
        referee.power_buffer = read16(p + 8); referee.heat_17mm = read16(p + 10);
        referee.heat_ms = now; referee.heat_seen = 1; referee.heat_sequence++;
        BSP_Unlock(mask);
        return;
    case 0x0207:
        if (size != 7U) break;
        float speed;
        memcpy(&speed, p + 3, sizeof(float));
        if (!isfinite(speed) || speed < 0) break;
        referee.bullet_type = p[0]; referee.shooter_id = p[1]; referee.shoot_frequency = p[2];
        referee.bullet_speed = speed;
        referee.shoot_ms = now;
        BSP_Unlock(mask);
        return;
    default:
        referee_diagnostics.unsupported++;
        BSP_Unlock(mask);
        return;
    }
    referee_diagnostics.length_error++;
    /* A CRC-valid but incompatible layout must not retain permission to shoot. */
    if (cmd == 0x0201U) referee.status_seen = 0;
    if (cmd == 0x0202U) referee.heat_seen = 0;
    BSP_Unlock(mask);
}
__weak void Referee_OnFrame(uint16_t command, const uint8_t *payload, uint16_t length)
{
    (void)command; (void)payload; (void)length;
}
void Referee_PushBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL) return;
    for (uint16_t i = 0; i < length; ++i) {
        if (used == REF_FRAME_CAPACITY) {
            discard(1);
            referee_diagnostics.length_error++;
        }
        frame[used++] = data[i];

        while (used >= 5U) {
            if (frame[0] != 0xA5U) { discard(1); continue; }
            if (!Verify_CRC8_Check_Sum(frame, 5)) {
                referee_diagnostics.crc_error++; discard(1); continue;
            }
            uint16_t payload = read16(frame + 1);
            if (payload > REF_FRAME_CAPACITY - 9U) {
                referee_diagnostics.length_error++; discard(1); continue;
            }
            uint16_t total = (uint16_t)(payload + 9U);
            if (used < total) break;
            if (!Verify_CRC16_Check_Sum(frame, total)) {
                referee_diagnostics.crc_error++; discard(1); continue;
            }
            uint16_t cmd = read16(frame + 5);
            solve(cmd, frame + 7, payload);
            referee_diagnostics.accepted++;
            Referee_OnFrame(cmd, frame + 7, payload);
            discard(total);
        }
    }
}
void Referee_Get(Referee_Data_t *data)
{
    uint32_t mask = BSP_Lock();
    *data = referee;
    BSP_Unlock(mask);
}
uint8_t Referee_IsOnline(void)
{
    Referee_Data_t value;
    Referee_Get(&value);
    return (uint8_t)(value.frame_seen &&
        (uint32_t)(HAL_GetTick() - value.frame_ms) <= GIMBAL_REFEREE_TIMEOUT_MS);
}
HAL_StatusTypeDef Referee_SendInteraction(uint16_t content_id, uint16_t receiver,
                                         const uint8_t *payload, uint16_t length)
{
    uint8_t tx[128];
    Referee_Data_t value;
    Referee_Get(&value);
    if (!value.status_seen || value.robot_id == 0U || !Referee_IsOnline() ||
        length > sizeof(tx) - 15U || (length != 0U && payload == NULL)) return HAL_ERROR;
    uint16_t total = (uint16_t)(15U + length);
    tx[0] = 0xA5; write16(tx + 1, (uint16_t)(length + 6U));
    tx[3] = sequence++;
    Append_CRC8_Check_Sum(tx, 5);
    write16(tx + 5, 0x0301U); write16(tx + 7, content_id);
    write16(tx + 9, value.robot_id); write16(tx + 11, receiver);
    if (length != 0U) memcpy(tx + 13, payload, length);
    Append_CRC16_Check_Sum(tx, total);
    return BSP_Referee_Transmit(tx, total);
}


