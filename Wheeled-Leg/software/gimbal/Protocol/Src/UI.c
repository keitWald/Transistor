#include "UI.h"
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "Dial.h"
#include "FrictionWheel.h"
#include "Gimbal_Config.h"
#include "Protocol_Referee.h"
#include "SBUS.h"
#include <stdio.h>
#include <string.h>

static uint8_t stage, robot;
static volatile uint8_t refresh_requested;
static uint32_t last_ms;
static void le32(uint8_t *p, uint32_t word) {
  p[0] = (uint8_t)word;
  p[1] = (uint8_t)(word >> 8);
  p[2] = (uint8_t)(word >> 16);
  p[3] = (uint8_t)(word >> 24);
}
static void graphic(uint8_t data[15], const char name[3], uint8_t operation,
                    uint8_t type, uint8_t color, unsigned start_angle,
                    unsigned end_angle, unsigned width, unsigned x, unsigned y,
                    unsigned end_x, unsigned end_y) {
  memcpy(data, name, 3);
  le32(data + 3, (uint32_t)operation | (uint32_t)type << 3 |
                     (uint32_t)color << 10 | start_angle << 14 |
                     end_angle << 23);
  le32(data + 7, width | x << 10 | y << 21);
  le32(data + 11, end_x << 10 | end_y << 21);
}
void UI_Init(void) {
  stage = robot = 0;
  last_ms = HAL_GetTick() - GIMBAL_UI_PERIOD_MS;
}
void UI_RequestRefresh(void) { refresh_requested = 1; }
void UI_Update(uint32_t now) {
  if ((uint32_t)(now - last_ms) < GIMBAL_UI_PERIOD_MS)
    return;
  last_ms = now;
    if (refresh_requested) { refresh_requested = 0; stage = 0; }
  Referee_Data_t value;
  Referee_Get(&value);
  if (!value.status_seen ||
      (uint32_t)(now - value.status_ms) > GIMBAL_REFEREE_TIMEOUT_MS)
    return;

  if (!((value.robot_id >= 1U && value.robot_id <= 6U) ||
        (value.robot_id >= 101U && value.robot_id <= 106U)))
    return;

  if (robot != value.robot_id) {
    robot = value.robot_id;
    stage = 0;
  }
  uint16_t client = (uint16_t)(0x100U + robot);
  if (stage == 0U) {
    uint8_t lines[30];
    graphic(lines, "GL1", 1, 0, 2, 0, 0, 2, 940, 540, 980, 540);
    graphic(lines + 15, "GL2", 1, 0, 2, 0, 0, 2, 960, 520, 960, 560);
    if (Referee_SendInteraction(0x0102, client, lines, sizeof(lines)) == HAL_OK)
      stage = 1;
  } else {
    uint8_t text[45] = {0};
    char label[31];
    Cloud_t cloud;
    Fric_Data_t friction;
    Dial_Data_t dial;
    SBUS_RevPack_t remote;
    Cloud_Get(&cloud);
    Fric_Get(&friction);
    Dial_Get(&dial);
    SBUS_Get(&remote);
    int degrees = (int)(cloud.Pitch_Raw * 57.2957795f);
    int count = snprintf(label, sizeof(label), "P:%+4d F:%u D:%u R:%u C:%u",
                         degrees, (unsigned)friction.Fric_Switch,
                         (unsigned)(dial.requested_rpm != 0),
                         (unsigned)remote.valid, (unsigned)Board1_IsOnline());
    if (count < 0)
      return;
    if (count > 30)
      count = 30;
    graphic(text, "GST", stage == 1U ? 1 : 2, 7, remote.armed ? 2 : 1, 18,
            (unsigned)count, 2, 60, 850, 0, 0);
    memcpy(text + 15, label, (size_t)count);
    if (Referee_SendInteraction(0x0110, client, text, sizeof(text)) == HAL_OK)
      stage = 2;
  }
}

