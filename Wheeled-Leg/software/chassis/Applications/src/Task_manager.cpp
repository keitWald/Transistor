#include "Task_manager.h"

#include <math.h>

#include "Chassis.h"
#include "Cloud.h"
#include "M3508.h"
#include "SBUS.h"
#include "SDM02.h"
#include "Saber_C3.h"
#include "board_comm.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "vofa.h"

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
void ObserverInit() {
    chassis.SpeedEstInit();
}
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
void Cloud_ControlTask() {
    Cloud.Cloud_Sport_Out();
}
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
void ChassisTask() {
    chassis.Controller();
}

/** Enable SDM02 ranging only during the pre-jump compression/preparation. */
void SDM02Task() {
    const bool jump_prepare = chassis.GetRobotStatus() == STATE_JUMPING && chassis.GetJumpSubStatus() == JUMP_COMPRESS;
    SDM02_Update(jump_prepare ? 1U : 0U);
}

/**
 * @brief 观测机身姿态，包括腿部解算
 * @param
 * @param
 * @param
 */
void ObserveTask() {
    chassis.Observe();
}
/**
 * @brief 板间通讯
 * @param
 * @param
 * @param
 */
void boardCommunicateTask() {
    Board2_FUN.Board2_To_1();
}
/**
 * @brief 上位机查看数据
 * @param
 * @param
 * @param
 */
void vofaTask() {
    // Direct pivot diagnostics. Keep all channels in physical units so the
    // source of Pitch drift can be separated from controller decomposition.
    Vofa.data[0] = INS.Pitch;
    Vofa.data[1] = (float)chassis.GetRobotStatus();
    Vofa.data[2] = INS.Gyro[1];
    Vofa.data[3] = chassis.GetTargetYawRate();
    Vofa.data[4] = INS.YawSpeed;
    Vofa.data[5] = chassis.GetNormalCenterSpeed();
    Vofa.data[6] = chassis.GetNormalFinalWheelCommonTorque();
    Vofa.data[7] = 0.5f * (chassis.GetLeftLegTor() + chassis.GetRightLegTor());
    Vofa.data[8] = (float)chassis.GetStepUpSubStatus();  // 上台阶子状态（VOFA 调试）
    Vofa.data[9] = fabsf(INS.Gyro[1]) / STEP_UP_IMPACT_GYRO_TH;  // 条件1，抖动
    Vofa.data[10] = chassis.GetStepUpWheelDecel() / STEP_UP_IMPACT_STALL_DECEL_TH;  // 条件2，骤降
    Vofa.data[11] = (fabsf(chassis.left_wheel.Get_Now_Torque()) + fabsf(chassis.right_wheel.Get_Now_Torque())) /
                    STEP_UP_IMPACT_TORQUE_TH;  // 条件3，扭矩
    Vofa.data[12] = chassis.GetStepUpImpactConfidence();  // 三信号加权置信度
}
