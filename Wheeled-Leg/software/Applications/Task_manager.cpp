#include "Task_manager.h"
#include "Chassis.h"
#include "Cloud.h"
#include "M3508.h"
#include "SBUS.h"
#include "Saber_C3.h"
#include "board_comm.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "vofa.h"
#include <math.h>

/**DEBUG**/
float joint_pos;
/**DEBUG**/

/**
 * @brief 底盘初始化
 * @param
 * @param
 * @param
 */
void ChassisInit() {
  chassis.MotorInit();
  chassis.PidInit();
  chassis.StatusInit();
}
/**
 * @brief 观测器初始化
 * @param
 * @param
 * @param
 */
void ObserverInit() { chassis.SpeedEstInit(); }
/**
 * @brief 云台部分初始化
 * @param
 * @param
 * @param
 */
void CloudInit() {
  Cloud.Motor_Init();
  Cloud.Cloud_Init();
}
/**
 * @brief 云台控制指令下发
 * @param
 * @param
 * @param
 */
void Cloud_ControlTask() { Cloud.Cloud_Sport_Out(); }
/**
 * @brief 达妙电机控制指令下发
 * @param
 * @param
 * @param
 */
void DM_MotorTask() {

  chassis.lf_joint_.DM_8009P_Ctrl();
  chassis.rf_joint_.DM_8009P_Ctrl();
  chassis.lb_joint_.DM_8009P_Ctrl();
  chassis.rb_joint_.DM_8009P_Ctrl();
}

/**
 * @brief 轮毂电机控制指令下发
 * @param
 * @param
 * @param
 */
void WheelMotorTask() {
  chassis.left_wheel.Ctrl();
  chassis.right_wheel.Ctrl();
  Class_Motor_3508::Set_Current();
}

/**
 * @brief 主控制计算循环
 * @param
 * @param
 * @param
 */
void ChassisTask() { chassis.Controller(); }

/**
 * @brief 观测机身姿态，包括腿部解算
 * @param
 * @param
 * @param
 */
void ObserveTask() { chassis.Observe(); }
/**
 * @brief 板间通讯
 * @param
 * @param
 * @param
 */
void boardCommunicateTask() { Board2_FUN.Board2_To_1(); }
/**
 * @brief 上位机查看数据
 * @param
 * @param
 * @param
 */
void vofaTask() {
  // Fixed jump/landing map.  Never remap on JUMP -> NORMAL/ESTOP so touchdown
  // and the following hand-off remain directly comparable.
  Vofa.data[0] = (float)chassis.GetRobotStatus();
  Vofa.data[1] = (float)chassis.GetJumpSubStatus();
  Vofa.data[2] = chassis.GetJumpBothContactLatched() ? 1.0f : 0.0f;
  Vofa.data[3] = (float)chassis.GetEstopReason();
  Vofa.data[4] = (float)chassis.GetDmFaultMask();
  Vofa.data[5] = INS.Pitch;
  Vofa.data[6] = INS.Gyro[1];
  Vofa.data[7] = (float)chassis.GetDmOfflineMask();
  Vofa.data[8] = (float)chassis.GetDmDisabledMask();
  Vofa.data[9] = chassis.GetJumpLandingWorldThetaRef();
  Vofa.data[10] = chassis.left_leg_.GetLegLen();
  Vofa.data[11] = chassis.right_leg_.GetLegLen();
  Vofa.data[12] = chassis.GetJumpCapturedWheelSpeed();
  Vofa.data[13] = chassis.GetJumpActiveWheelSpeedRef();
  Vofa.data[14] = chassis.left_wheel.Get_Now_Omega();
  Vofa.data[15] = chassis.right_wheel.Get_Now_Omega();
}
