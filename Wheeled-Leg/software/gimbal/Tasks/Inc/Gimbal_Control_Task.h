#ifndef GIMBAL_CONTROL_TASK_H
#define GIMBAL_CONTROL_TASK_H
#include "cmsis_os2.h"
#define GIMBAL_READY_FLAG 1U
extern osEventFlagsId_t gimbal_ready_event;
void Gimbal_Control_Task(void *argument);
void Gimbal_WaitReady(void);
void Gimbal_DelayUntil(uint32_t *wake, uint32_t milliseconds);
#endif

