#ifndef GIMBAL_PROTOCOL_REFEREE_H
#define GIMBAL_PROTOCOL_REFEREE_H
#include <stdint.h>
#include "main.h"
/* Payload layouts follow the existing Protocol_Judgement.h v1.6.1 subset.
   Wire data is decoded bytewise; no compiler-specific packed structs. */
typedef struct {
    uint8_t robot_id, robot_level, game_progress, power_outputs;
    uint8_t bullet_type, shooter_id, shoot_frequency;
    uint16_t hp, hp_max, heat_limit, cooling_per_second, chassis_power_limit;
    uint16_t voltage_mv, current_ma, power_buffer, heat_17mm;
    uint16_t stage_remaining_seconds;
    float chassis_power, bullet_speed;
    uint32_t status_ms, heat_ms, shoot_ms, frame_ms;
    uint32_t status_sequence, heat_sequence;
    uint8_t status_seen, heat_seen, frame_seen;
} Referee_Data_t;
typedef struct {
    uint32_t crc_error, length_error, accepted, unsupported;
} Referee_Diagnostics_t;
extern volatile Referee_Diagnostics_t referee_diagnostics;
void Referee_Init(void);
void Referee_ResetParser(void);
void Referee_PushBytes(const uint8_t *data, uint16_t length);
void Referee_Get(Referee_Data_t *data);
uint8_t Referee_IsOnline(void);
HAL_StatusTypeDef Referee_SendInteraction(uint16_t content_id, uint16_t receiver,
                                         const uint8_t *payload, uint16_t length);
/* Called in the referee task for every valid frame, including unhandled commands.
   Payload pointer is valid only until this callback returns. */
void Referee_OnFrame(uint16_t command, const uint8_t *payload, uint16_t length);
#endif

