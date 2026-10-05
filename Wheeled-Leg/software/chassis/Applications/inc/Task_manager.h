#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "board_comm.h"
extern void motor_init();
extern void DM_MotorTask();
extern void WheelMotorTask();
extern void YawMotorTask();
extern void ChassisTask();
extern void SDM02Task();
extern void boardCommunicateTask();
extern void vofaTask();
extern void ChassisInit();
extern void CloudInit();
extern void Cloud_ControlTask();
extern void ObserveTask();
extern void ObserverInit();
#ifdef __cplusplus
}
#endif

#endif
