#ifndef GIMBAL_UI_H
#define GIMBAL_UI_H
#include <stdint.h>
void UI_Init(void);
void UI_RequestRefresh(void);
void UI_Update(uint32_t now);
#endif

