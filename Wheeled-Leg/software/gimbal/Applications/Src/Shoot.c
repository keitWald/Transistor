#include "Shoot.h"
#include "FrictionWheel.h"
#include "Dial.h"
#include "Protocol_Referee.h"
#include "Gimbal_Config.h"
#include "BSP_Lock.h"

static Shoot_Data_t state;
static uint32_t last_ms, heat_sequence;
void Shoot_Init(void)
{
    state = (Shoot_Data_t){0};
    last_ms = HAL_GetTick(); heat_sequence = 0;
    Fric_Init(); 
    Dial_Init();
}
void Shoot_Processing(const SBUS_RevPack_t *remote, float dt)
{
    Fric_Processing(remote, dt);
    Fric_Data_t friction;
    Referee_Data_t referee;
    Fric_Get(&friction); 
    Referee_Get(&referee);
    uint32_t now = HAL_GetTick();
    uint8_t fresh = (uint8_t)(referee.status_seen && referee.heat_seen &&
        (uint32_t)(now - referee.status_ms) <= GIMBAL_REFEREE_TIMEOUT_MS &&
        (uint32_t)(now - referee.heat_ms) <= GIMBAL_REFEREE_TIMEOUT_MS);
    uint32_t mask = BSP_Lock();
    float elapsed = (float)(uint32_t)(now - last_ms) / 1000.0f;
    last_ms = now;
    if (fresh) state.estimated_heat -= referee.cooling_per_second * elapsed;
    if (state.estimated_heat < 0) state.estimated_heat = 0;
    if (referee.heat_seen && heat_sequence != referee.heat_sequence) {
        heat_sequence = referee.heat_sequence;
        if (state.estimated_heat < referee.heat_17mm)
            state.estimated_heat = referee.heat_17mm;
    }
    state.Heat_Now = referee.heat_17mm; 
    state.Heat_Limit = referee.heat_limit;
    state.heat_permitted = (uint8_t)(fresh && referee.hp != 0U &&
        (referee.power_outputs & 4U) != 0U &&
        state.estimated_heat + GIMBAL_HEAT_RESERVE < referee.heat_limit);
    state.Shoot_Switch = (uint8_t)(remote->armed && remote->feed_on && friction.Fric_Ready &&
                                  state.heat_permitted);
    uint8_t feed = state.Shoot_Switch;
    BSP_Unlock(mask);
    Dial_Processing(feed, GIMBAL_DIAL_RPM);
}
void Shoot_ReportShot(void)
{
    uint32_t mask = BSP_Lock();
    state.estimated_heat += GIMBAL_SHOT_HEAT;
    BSP_Unlock(mask);
}
void Shoot_Get(Shoot_Data_t *value)
{
    uint32_t mask = BSP_Lock();
    *value = state;
    BSP_Unlock(mask);
}


