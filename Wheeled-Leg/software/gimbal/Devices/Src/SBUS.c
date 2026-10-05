#include "SBUS.h"
#include "Gimbal_Config.h"
#include "BSP_Lock.h"
#include <string.h>
#include <stdlib.h>
/* Mapping and Ramp_Handle below are reused from the chassis SBUS driver. */
static const uint16_t SBUS_MIN=321, SBUS_MID=992, SBUS_MAX=1663;
static float map_sbus_to_range(uint16_t value, float min_out, float max_out);
static float Ramp_Handle(float current, float target, float ramp_rate, float dt);


static SBUS_RevPack_t command;
static uint8_t frame[SBUS_DATA_SIZE], used, safe_seen, arm_frames;
_Static_assert(GIMBAL_CH_YAW < 16 && GIMBAL_CH_FORWARD < 16 && GIMBAL_CH_LATERAL < 16 &&
               GIMBAL_CH_PITCH < 16 && GIMBAL_CH_ARM < 16 && GIMBAL_CH_FRICTION < 16 &&
               GIMBAL_CH_FEED < 16 && GIMBAL_CH_UI_REFRESH < 16, "SBUS channel indices");

void SBUS_Reset(void)
{
    uint32_t mask = BSP_Lock();
    command.valid = command.armed = command.friction_on = command.feed_on = 0;
    command.speed = command.w_speed = command.yaw_speed = command.pitch_rate = 0;
    command.ui_refresh = 0; command.yaw_speed_mapped = 0; command.len = MID_LEN;
    safe_seen = arm_frames = used = 0;
    BSP_Unlock(mask);
}
void SBUS_Init(void)
{
    memset(&command, 0, sizeof(command));
    SBUS_Reset();
}
static uint8_t switch_level(uint16_t value)
{
    if (value >= GIMBAL_SBUS_MIN - 50U && value <= GIMBAL_SBUS_MIN + 50U) return 1;
    if (value >= GIMBAL_SBUS_MID - 50U && value <= GIMBAL_SBUS_MID + 50U) return 2;
    if (value >= GIMBAL_SBUS_MAX - 50U && value <= GIMBAL_SBUS_MAX + 50U) return 3;
    return 0;
}
static float map_to_2levels(int16_t sbus_val) {
  if (sbus_val <= SBUS_MIN + TOLERANCE) {
    return 1;
  } else if (sbus_val >= SBUS_MAX - TOLERANCE) {
    return 2;
  }
  return 0;
}

static int decode_frame(void)
{
    if (frame[0] != 0x0FU ||
        !(frame[24] == 0U || (frame[24] & 0x0FU) == 0x04U)) return 0;
    if ((frame[23] & 0x0CU) != 0U) {
        SBUS_Reset(); /* Receiver failsafe/frame-loss stops outputs immediately. */
        return 1;
    }
    SBUS_RevPack_t next = {0};
    for (unsigned i = 0; i < 16; ++i) {
        unsigned bit = i * 11U, byte = 1U + bit / 8U, shift = bit % 8U;
        uint32_t packed = (uint32_t)frame[byte] | (uint32_t)frame[byte + 1U] << 8;
        if (byte + 2U <= 22U) packed |= (uint32_t)frame[byte + 2U] << 16;
        next.channel[i] = (uint16_t)((packed >> shift) & 0x7FFU);
    }
    /* Only the assigned channels have the chassis calibration contract. */
    const uint8_t assigned[] = { GIMBAL_CH_YAW, GIMBAL_CH_FORWARD, GIMBAL_CH_PITCH,
        GIMBAL_CH_LATERAL, GIMBAL_CH_ARM, GIMBAL_CH_FRICTION, GIMBAL_CH_FEED,
        GIMBAL_CH_UI_REFRESH };
    for (unsigned i = 0; i < sizeof(assigned); ++i) {
        uint16_t value = next.channel[assigned[i]];
        if (value < GIMBAL_SBUS_MIN - 50U || value > GIMBAL_SBUS_MAX + 50U) {
            SBUS_Reset();
            return 1;
        }
    }
    uint32_t mask = BSP_Lock();
    if (!command.valid || (uint32_t)(HAL_GetTick() - command.last_ms) > GIMBAL_SBUS_TIMEOUT_MS)
        safe_seen = arm_frames = 0;
    uint8_t level = switch_level(next.channel[GIMBAL_CH_ARM]);
    if (level != 2U) {
        safe_seen = (uint8_t)(level == 1U || level == 3U);
        arm_frames = 0;
    } else if (safe_seen && arm_frames < 2U) arm_frames++;
    next.armed = (uint8_t)(safe_seen && arm_frames >= 2U && level == 2U);
    next.friction_on = (uint8_t)(next.armed &&
                           switch_level(next.channel[GIMBAL_CH_FRICTION]) == 3U);
    next.feed_on = (uint8_t)(next.friction_on &&
                           switch_level(next.channel[GIMBAL_CH_FEED]) == 3U);
    next.ui_refresh = (uint8_t)(switch_level(next.channel[GIMBAL_CH_UI_REFRESH]) == 3U);
    float dt = (float)(uint32_t)(HAL_GetTick() - command.last_ms) / 1000.0f;
    if (dt <= 0 || dt > 0.1f) dt = 0.02f;
    next.len = Ramp_Handle(command.len,
        -map_sbus_to_range(next.channel[12], -MAX_LEN, MAX_LEN) + MID_LEN,
        RAMP_RATE_LEN, dt);
    next.torque = Ramp_Handle(command.torque,
        map_sbus_to_range(next.channel[2], -TORQUE_MAX, TORQUE_MAX), RAMP_RATE_TORQUE, dt);
    next.jump_flag = map_to_2levels((int16_t)next.channel[9]);
    next.reset_flag = map_to_2levels((int16_t)next.channel[11]);
    next.step_up_flag = map_to_2levels((int16_t)next.channel[10]);
    next.status_flag = level; /* Chassis: 1=recover, 2=normal, 3=estop. */
    if (next.armed) {
        next.speed = Ramp_Handle(command.speed,
            map_sbus_to_range(next.channel[GIMBAL_CH_FORWARD], -SPEED_MAX, SPEED_MAX),
            RAMP_RATE_SPEED, dt);
        next.w_speed = Ramp_Handle(command.w_speed,
            map_sbus_to_range(next.channel[GIMBAL_CH_LATERAL], -W_SPEED_MAX, W_SPEED_MAX),
            RAMP_RATE_W_SPEED, dt);
        next.yaw_speed_mapped = -map_sbus_to_range(next.channel[GIMBAL_CH_YAW],
                                                   -YAW_SPEED_MAX, YAW_SPEED_MAX);
        next.yaw_speed = Ramp_Handle(command.yaw_speed, next.yaw_speed_mapped,
                                     RAMP_RATE_YAW, dt);
        next.pitch_rate = map_sbus_to_range(next.channel[GIMBAL_CH_PITCH],
                           -GIMBAL_PITCH_RATE_RAD_S, GIMBAL_PITCH_RATE_RAD_S);
    }
    next.valid = 1; next.last_ms = HAL_GetTick();
    command = next;
    BSP_Unlock(mask);
    return 1;
}
void SBUS_PushBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL) return;
    for (uint16_t i = 0; i < length; ++i) {
        if (used == 0U && data[i] != 0x0FU) continue;
        frame[used++] = data[i];
        if (used == SBUS_DATA_SIZE) {
            if (decode_frame()) used = 0;
            else {
                memmove(frame, frame + 1, SBUS_DATA_SIZE - 1U);
                used--;
                while (used != 0U && frame[0] != 0x0FU) {
                    memmove(frame, frame + 1, --used);
                }
            }
        }
    }
}
void SBUS_Get(SBUS_RevPack_t *result)
{
    uint32_t mask = BSP_Lock();
    *result = command;
    if (!result->valid || (uint32_t)(HAL_GetTick() - result->last_ms) > GIMBAL_SBUS_TIMEOUT_MS) {
        command.valid = 0;
        safe_seen = arm_frames = 0; /* Reconnection in the armed detent cannot restart outputs. */
        result->valid = result->armed = result->friction_on = result->feed_on = 0;
        result->speed = result->w_speed = result->yaw_speed = result->pitch_rate = 0;
        result->yaw_speed_mapped = result->status_flag = 0;
    }
    BSP_Unlock(mask);
}
uint8_t SBUS_IsLinkLost(void)
{
    SBUS_RevPack_t value;
    SBUS_Get(&value);
    return (uint8_t)!value.valid;
}



static float map_sbus_to_range(uint16_t sbus_value, float min_out,
                               float max_out) {
  if (sbus_value < SBUS_MIN)
    sbus_value = SBUS_MIN;
  if (sbus_value > SBUS_MAX)
    sbus_value = SBUS_MAX;

  if (abs(sbus_value - SBUS_MID) <= DEADZONE_THRESHOLD) {
    return 0.0f;
  }

  if (sbus_value > SBUS_MID) {
    uint16_t effective_min = SBUS_MID + DEADZONE_THRESHOLD;
    return (float)(sbus_value - effective_min) /
           (float)(SBUS_MAX - effective_min) * max_out;
  } else {
    uint16_t effective_max = SBUS_MID - DEADZONE_THRESHOLD;
    // 注意分母是 (SBUS_MIN - effective_max)，这是一个负数，与min_out的符号抵消
    return (float)(sbus_value - effective_max) /
           (float)(SBUS_MIN - effective_max) * min_out;
  }
}

/**
 * @brief 斜坡函数，使当前值平滑地趋近于目标值。
 */
static float Ramp_Handle(float current, float target, float ramp_rate,
                         float dt) {
  float max_change = ramp_rate * dt;
  float error = target - current;

  if (error > max_change) {
    return current + max_change;
  } else if (error < -max_change) {
    return current - max_change;
  } else {
    return target;
  }
}


