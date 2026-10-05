#ifndef GIMBAL_SBUS_H
#define GIMBAL_SBUS_H
#include "main.h"
/* Physical units, mappings and ramps adapted from chassis/Devices/src/SBUS.c. */
#define SBUS_DATA_SIZE 25U
#define DEADZONE_THRESHOLD 50.0f
#define SPEED_MAX 2.5f
#define W_SPEED_MAX 2.0f
#define YAW_SPEED_MAX 6.0f
#define MID_LEN 0.33f
#define MAX_LEN 0.10f
#define TORQUE_MAX 2.0f
#define TOLERANCE 50
#define SBUS_LINK_TIMEOUT_MS 100U
#define RAMP_RATE_SPEED 1.75f
#define RAMP_RATE_W_SPEED 3.0f
#define RAMP_RATE_YAW 0.2f
#define RAMP_RATE_LEN 0.2f
#define RAMP_RATE_TORQUE 1.0f
typedef struct {
  float speed;            // 前进/后退速度 (m/s)
  float w_speed;          // 横移速度 (m/s)
  float len;              // 腿长 (m)
  float yaw_speed;        // 偏航角速度 (rad/s)
  float yaw_speed_mapped; // dead-zone linear mapping before the yaw ramp
  float jump_flag;        // 跳跃开关两档（1→2 上升沿触发跳跃）
  float status_flag;      // Robot mode: 1=recover(自起), 2=normal(正常),
                          // 0/3=estop(急停)
  float step_up_flag;     // 上台阶开关（1=低位/关闭，2=高位/使能）
  float reset_flag;       // 整车复位标志
  float torque;
    uint16_t channel[16];
    float pitch_rate;
    uint32_t last_ms;
    uint8_t valid, armed, friction_on, feed_on, ui_refresh;
} SBUS_RevPack_t;
void SBUS_Init(void);
void SBUS_Reset(void);
void SBUS_PushBytes(const uint8_t *data, uint16_t length);
void SBUS_Get(SBUS_RevPack_t *command);
uint8_t SBUS_IsLinkLost(void);
#endif
