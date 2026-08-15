#include "Chassis.h"
#include "BSP_fdcan.h"
#include "SBUS.h"
#include "board_comm.h"
#include "bsp_dwt.h"
#include "fdcan.h"
#include "ins.h"
#include "stm32h7xx_hal_spi.h"
#include "usart.h"

/*************const***************/
const float k_gravity_comp = 5.3445f * 9.8f;
const float k_wheel_radius = 0.06;
const float k_phi1_bias = 0.9124f + 3.1415f;
const float k_phi4_bias = 3.1415f;

const float k_lf_joint_bias = 0.4832;
const float k_lb_joint_bias = 6.0972;
const float k_rf_joint_bias = 1.6409;
const float k_rb_joint_bias = 2.9341;

const float k_jump_force = 250.0f;
const float k_jump_time = 0.1f;
const float k_retract_fast_force = -230.0f;
const float k_retract_near_force = -190.0f;
const float k_retract_near_length = 0.25f;
const float k_retract_time = 0.15f;

const float leg_max = 0.4357; // 0.4057
const float leg_min = 0.2226;

static float target_yaw;

float test_data = 0;
float input_angel = 0;

/***********************************/

namespace {

float ClampAbs(float value, float limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

float ClampRange(float value, float min_value, float max_value) {
  if (value > max_value)
    return max_value;
  if (value < min_value)
    return min_value;
  return value;
}

float SlewTowards(float current, float target, float max_step) {
  if (target > current + max_step)
    return current + max_step;
  if (target < current - max_step)
    return current - max_step;
  return target;
}

} // namespace

balance_Chassis chassis;
/**
 * @brief 腿部电机初始化
 * @param
 * @param
 * @param
 */
void balance_Chassis::MotorInit() {
  lf_joint_.Init(&hfdcan1, 0x01, 0x01, Motor_DM_Control_Method_NORMAL_MIT,
                 P_MAX, V_MAX, T_MAX, I_MAX);
  lb_joint_.Init(&hfdcan1, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT,
                 P_MAX, V_MAX, T_MAX, I_MAX);
  rf_joint_.Init(&hfdcan1, 0x03, 0x03, Motor_DM_Control_Method_NORMAL_MIT,
                 P_MAX, V_MAX, T_MAX, I_MAX);
  rb_joint_.Init(&hfdcan1, 0x04, 0x04, Motor_DM_Control_Method_NORMAL_MIT,
                 P_MAX, V_MAX, T_MAX, I_MAX);
  left_wheel.Init(CAN_Motor_ID_0x201, Control_Method_TORQUE);
  right_wheel.Init(CAN_Motor_ID_0x202, Control_Method_TORQUE);
  // 设置电机零点位置，只在需要重新设置零点时使用
  //   lf_joint_.Save_Pos_Zero();
  //   lb_joint_.Save_Pos_Zero();
  //  rf_joint_.Save_Pos_Zero();
  //  rb_joint_.Save_Pos_Zero();
  lf_joint_.Enable();
  lb_joint_.Enable();
  rf_joint_.Enable();
  rb_joint_.Enable();
}
/**
 * @brief 底盘状态初始化
 * @param
 * @param
 * @param
 */
void balance_Chassis::StatusInit() {
  dist_ = 0;
  dist_m_ = 0.0f;
  target_dist_ = 0;
  rotation_ = 0.0f;
  target_rotation_ = 0.0f;
  heading_last_ = 0.0f;
  heading_initialized_ = false;
  translation_command_active_ = false;
  yaw_command_active_ = false;
  normal_pivot_yaw_rate_ref_ = 0.0f;
  normal_pivot_yaw_torque_cmd_ = 0.0f;
  normal_pivot_center_trim_torque_ = 0.0f;
  normal_pivot_center_prev_speed_ = 0.0f;
  normal_zero_speed_trim_torque_ = 0.0f;
  normal_speed_ref_ = 0.0f;
  normal_wheel_center_speed_ = 0.0f;
  normal_wheel_diff_speed_ = 0.0f;
  normal_pivot_leg_sync_force_ = 0.0f;
  normal_pivot_leg_sync_integral_ = 0.0f;
  normal_pivot_left_speed_integral_ = 0.0f;
  normal_pivot_right_speed_integral_ = 0.0f;
  normal_pivot_pitch_integral_ = 0.0f;
  normal_handoff_leg_grace_count_ = 0;
  dm_fault_latched_mask_ = 0;
  dm_offline_latched_mask_ = 0;
  dm_disabled_latched_mask_ = 0;
  dm_estop_confirmed_ = false;
  dm_enable_wait_count_ = 0;
  roll_force_cmd_ = 0.0f;
  normal_pivot_roll_integral_ = 0.0f;
  roll_len_delta_cmd_ = 0.0f;
  off_ground_enter_count_ = 0;
  off_ground_exit_count_ = 0;
  // --- 状态机初始化 ---
#if CHASSIS_JOINT_DEBUG_ENABLE && !CHASSIS_TEACH_ENABLE
  robot_status = STATE_JOINT_DEBUG;
  return;
#endif
  // 默认从“正常”状态开始
  recover_dynamic_len_ref_ = recover_final_target_len;
  robot_status = STATE_NORMAL;
  jump_status = JUMP_NONE;
  jump_state_ = false;
  last_jump_flag_ = 0;
  // --- 计时器初始化 ---
  recover_timer = 0;
  fall_detect_count_ = 0;
  balance_count = 0;
  align_count = 0;
  shoutui_count = 0;
  jump_timer = 0;
  jump_length_ready_count_ = 0;
  jump_retract_hold_count_ = 0;
  jump_landing_ready_count_ = 0;
  jump_normal_handoff_count_ = 0;
  jump_wheel_integral_l_ = 0.0f;
  jump_wheel_integral_r_ = 0.0f;
  jump_wheel_speed_ref_l_ = 0.0f;
  jump_wheel_speed_ref_r_ = 0.0f;
  jump_active_wheel_ref_l_ = 0.0f;
  jump_active_wheel_ref_r_ = 0.0f;
  jump_landing_speed_scale_ = 1.0f;
  jump_land_phi0_ref_l_ = 0.0f;
  jump_land_phi0_ref_r_ = 0.0f;
  jump_landing_brake_ref_l_ = 0.0f;
  jump_landing_brake_ref_r_ = 0.0f;
  jump_left_contact_count_ = jump_right_contact_count_ = 0U;
  jump_left_release_count_ = jump_right_release_count_ = 0U;
  jump_left_contact_ = jump_right_contact_ = false;
  jump_both_contact_latched_ = false;
  jump_landing_zero_cross_l_ = false;
  jump_landing_zero_cross_r_ = false;
  jump_pitch_integral_ = 0.0f;
  jump_roll_integral_ = 0.0f;
  jump_liftoff_seen_ = false;
  warming_counter_ = 0;
  recover_balance_cnt_ = 0;
  recover_leg_retracted_ = false;
  recover_failed_ = false;
  recover_request_latched_ = false;
  recover_sub_status_ = RECOVER_SETTLE;
  recover_sub_timer_ = 0;
  recover_sub_stable_count_ = 0;
  recover_center_wheel_count_ = 0;
  recover_capture_pulse_count_ = 0;
  recover_capture_pitch_sign_ = 0.0f;
  recover_pose_lost_count_ = 0;
  recover_sbus_lost_count_ = 0;
  estop_reason_ = 0;
  recover_sweep_phi_ref_ = 0.0f;
  recover_sweep_phi_goal_ = 0.0f;
  recover_wheel_torque_lpf_l_ = 0.0f;
  recover_wheel_torque_lpf_r_ = 0.0f;
  recover_joint_lift_torque_ = 0.0f;
  DaoDiFlg_ = true; // 上电默认认为倒地
  Z_Flag_ = false;
  Last_Z_Key_ = false;
}
/**
 * @brief 所有所需PID初始化
 * @param
 * @param
 * @param
 */
void balance_Chassis::PidInit() {
  // 腿长 PID、防劈叉 PID、roll PID 和髋关节角度 PID 初始化
  right_leg_len_.Init(500.0f, 0.0f, 50.0f, 400.0f, 0.001f);
  left_leg_len_.Init(500.0f, 0.0f, 50.0f, 400.0f, 0.001f);
  // Differentiate the measured leg length and filter both the derivative and
  // final force command.  The former implementation differentiated the error
  // without filtering at 1 kHz, amplifying encoder quantisation into leg shake.
  const uint8_t leg_pid_improvements =
      PID_DERIVATIVE_ON_MEASUREMENT | PID_DERIVATIVE_FILTER | PID_OUTPUT_FILTER;
  right_leg_len_.Inprovement(leg_pid_improvements, 0.0f, 0.0f, 0.0f, 0.005f,
                             0.015f);
  left_leg_len_.Inprovement(leg_pid_improvements, 0.0f, 0.0f, 0.0f, 0.005f,
                            0.015f);
  // recover 模式下的 phi0 双环：
  // 单环位置 PID 直接输出髋关节力矩；扫腿行程大，提高增益与限幅保证力矩足够。
  left_leg_phi0.Init(20.0f, 2.0f, 25.0f, 40.0f, 0.02f);
  right_leg_phi0.Init(20.0f, 2.0f, 25.0f, 40.0f, 0.02f);
  // 速度环 PID（当前恢复流程未使用，保留初始化）
  left_leg_phi0_speed_.Init(0.6f, 0.0f, 0.05f, 2.0f, 0.0f);
  right_leg_phi0_speed_.Init(0.6f, 0.0f, 0.05f, 2.0f, 0.0f);
  // 防劈叉 PID，双腿建模阶段暂不使用
  anti_crash_.Init(60.0f, 0.0f, 30.0f, 50.0f, 0.001f);
  // Keep this legacy/shared PID at its established values. NORMAL uses the
  // explicit controller in NormalCalc(), so tuning it cannot alter RECOVER.
  roll_comp_.Init(40.0f, 0.0f, 4.0f, 15.0f, kRollDeadBand);
}

/**
 * @brief 速度和加速度卡尔曼滤波观测器
 * @param
 * @param
 * @param
 */
void balance_Chassis::SpeedEstInit() {
  // 使用kf同时估计速度和加速度
  Kalman_Filter_Init(&kf, 2, 0, 2);
  float F[4] = {1, 0.002, 0, 1}; // A矩阵，F[0][1] 会在运行时按 dt 更新
  float Q[4] = {VEL_PROCESS_NOISE, 0, 0,
                ACC_PROCESS_NOISE}; // 状态变量过程噪声矩阵
  float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE}; // 测量噪声矩阵
  // float P[4] = {100000, 0, 0, 100000};//状态估计误差协方差矩阵
  float P[4] = {1, 0, 0, 1}; // 状态估计误差协方差矩阵
  float H[4] = {1, 0, 0, 1};
  memcpy(kf.F_data, F, sizeof(F));
  memcpy(kf.Q_data, Q, sizeof(Q));
  memcpy(kf.R_data, R, sizeof(R));
  memcpy(kf.P_data, P, sizeof(P));
  memcpy(kf.H_data, H, sizeof(H));
}
/**
 * @brief 腿部状态正运动学计算
 * @param
 * @param
 * @param
 */
void balance_Chassis::LegCalc() {
  left_leg_.SetBodyData(INS.Pitch, INS.MotionAccel_n[2], INS.Gyro[1]);
  right_leg_.SetBodyData(INS.Pitch, INS.MotionAccel_n[2], INS.Gyro[1]);
  left_leg_.SetLegData(lf_joint_.GetAngle(), lf_joint_.GetSpeed(),
                       lb_joint_.GetAngle(), lb_joint_.GetSpeed(),
                       lf_joint_.GetTor(), lb_joint_.GetTor());
  right_leg_.SetLegData(-rf_joint_.GetAngle(), -rf_joint_.GetSpeed(),
                        -rb_joint_.GetAngle(), -rb_joint_.GetSpeed(),
                        -rf_joint_.GetTor(), -rb_joint_.GetTor());

  left_leg_.LegCalc();
  right_leg_.LegCalc();
  left_leg_.DerivateCalc();
  right_leg_.DerivateCalc();
  left_leg_.Jacobian();
  right_leg_.Jacobian();
  left_leg_.LegForceCalc();
  right_leg_.LegForceCalc();
}
/**
 * @brief 主控制循环
 * @param
 * @param
 * @param
 */
void balance_Chassis::Controller() {
  SetState();
  ChassissControl();
  TorControl();
}
/**
 * @brief 兼容旧调用点，当前仅保留状态更新功能
 * @param
 * @param
 * @param
 */
void balance_Chassis::Rev() {
  // 旧接口保留；推荐直接调用 Controller()。
  SetState();
}

/**
 * @brief 重置起身状态相关的内部标志和计数器。
 */
void balance_Chassis::ResetRecoverState() {
  recover_timer = 0;
  balance_count = 0;
  align_count = 0;
  recover_align_count_ = 0;
  shoutui_count = 0;
  shoutui_count1 = 0;
  recover_state_ = false;
  recover_align_done_ = false;
  recover_phi0_ref_init_ = false;
  recover_phi0_unwrap_init_ = false;
  recover_phi0_ref_l_ = 0.0f;
  recover_phi0_ref_r_ = 0.0f;
  recover_phi0_unwrap_l_ = 0.0f;
  recover_phi0_unwrap_r_ = 0.0f;
  recover_phi0_last_l_ = 0.0f;
  recover_phi0_last_r_ = 0.0f;
  recover_phi0_wrap_l_ = 0;
  recover_phi0_wrap_r_ = 0;
  recover_enable_pending_ = false;
  recover_long_path_l_ = false;
  recover_long_path_r_ = false;
  recover_decided_l_ = false;
  recover_decided_r_ = false;
  recover_long_cross_zero_l_ = false;
  recover_long_cross_zero_r_ = false;
  recover_long_passed_pi_l_ = false;
  recover_long_passed_pi_r_ = false;
  recover_passed_pi_l_ = false;
  recover_passed_pi_r_ = false;
  recover_leg_retracted_ = false;
  recover_failed_ = false;
  recover_balance_cnt_ = 0;
  recover_yaw_cnt_ = 0;
  recover_sub_status_ = RECOVER_SETTLE;
  recover_sub_timer_ = 0;
  recover_sub_stable_count_ = 0;
  recover_center_wheel_count_ = 0;
  recover_capture_pulse_count_ = 0;
  recover_capture_pitch_sign_ = 0.0f;
  recover_pose_lost_count_ = 0;
  recover_sbus_lost_count_ = 0;
  recover_sweep_phi_ref_ = 0.0f;
  recover_sweep_phi_goal_ = 0.0f;
  recover_wheel_torque_lpf_l_ = 0.0f;
  recover_wheel_torque_lpf_r_ = 0.0f;
  recover_yaw_target_ = INS.Yaw; // 锁定起身目标朝向（参考山海机甲 Target_Yaw）
  left_leg_len_.Clear();
  right_leg_len_.Clear();
  left_leg_phi0.Clear();
  right_leg_phi0.Clear();
  left_leg_phi0_speed_.Clear();
  right_leg_phi0_speed_.Clear();
}

/**
 * @brief 参考山海机甲开源：使用IK + 关节角LQR 的收腿控制
 * @param leg 腿部VMC对象
 * @param target_phi0 目标摆杆角度
 * @param target_l0 目标腿长
 * @param T1 输出：关节1力矩
 * @param T2 输出：关节2力矩
 */
void balance_Chassis::RecoverLegJointLQR(Vmc &leg, float target_phi0,
                                         float target_l0, float &T1,
                                         float &T2) {
  // 1. IK 解算: phi0, L0 → phi1, phi4
  leg.SetLegIKTarget(target_phi0, target_l0);
  leg.LegInverseCalc();
  if (!leg.GetIKValid()) {
    // IK 无解时输出零力矩，等待下一周期重新计算
    T1 = 0.0f;
    T2 = 0.0f;
    return;
  }
  const float target_phi1 = leg.GetPhi1IK();
  const float target_phi4 = leg.GetPhi4IK();

  // 2. 关节角 LQR 控制（参考山海机甲 Leg_Controller_LengthLQR）
  // T1 = -K_FF - K1*(phi1 - target_phi1) - K2*dphi1
  // T2 = +K_FF - K1*(phi4 - target_phi4) - K2*dphi4
  const float phi1_err = leg.GetPhi1() - target_phi1;
  const float phi4_err = leg.GetPhi4() - target_phi4;

  // In the folded configuration, increasing leg length requires phi1 to
  // increase and phi4 to decrease.  The historical feed-forward signs were
  // intended for retracting a fallen leg and therefore cancelled nearly half
  // of the extension torque here.  Reverse only that feed-forward pair so it
  // assists the short-leg lift; the PD terms and damping remain unchanged.
  T1 = Leg_Controller_LQR_FeedForward - Leg_Controller_LQR_K1 * phi1_err -
       Leg_Controller_LQR_K2 * leg.GetWPhi1();
  T2 = -Leg_Controller_LQR_FeedForward - Leg_Controller_LQR_K1 * phi4_err -
       Leg_Controller_LQR_K2 * leg.GetWPhi4();
}

/**
 * @brief 急停控制：全停零力矩，所有电机全部停止
 * @note  不再分级阻尼/支撑；进入 ESTOP 后每拍直接对全部电机下发零力矩。
 *        warming_counter_ 仅作急停持续计数（调试观察），不影响输出。
 */
void balance_Chassis::WarmingMotorControl() {
  warming_counter_++;

  // 清空所有控制量（保证后续 TorControl 不产生额外输出）
  l_wheel_T_ = 0.0f;
  r_wheel_T_ = 0.0f;
  left_leg_F_ = 0.0f;
  right_leg_F_ = 0.0f;
  left_leg_T_ = 0.0f;
  right_leg_T_ = 0.0f;

  // 所有电机零力矩：轮组目标扭矩 0 + 4 个关节 MIT 0
  StopMotor();
}

void balance_Chassis::ConfigureRecoverLegControl(float current_phi0,
                                                 bool use_long_path,
                                                 bool passed_pi, Pid &phi0_pid,
                                                 float &leg_len_ref) {
  const float recover_phi0_ref = 0.5f * PI;
  const float recover_len_retracted = recover_final_target_len;
  const float recover_len_extended = recover_tmp_target_len;

  // 默认按“腿已经在身后”的情况处理：
  // 收腿，同时直接把 phi0 调到起身角。
  float phi0_measure_for_pid = current_phi0;
  float phi0_ref_for_pid = recover_phi0_ref;
  leg_len_ref = recover_len_retracted;

  // 如果这条腿原来在身前，而且还没绕到身后，
  // 就先执行“长路径绕后”这一段。
  if (use_long_path && !passed_pi) {
    leg_len_ref = recover_len_extended;
    if (current_phi0 < recover_phi0_ref) {
      phi0_measure_for_pid = current_phi0 + 2.0f * PI;
    }
  }

  phi0_pid.SetMeasure(phi0_measure_for_pid);
  phi0_pid.SetRef(phi0_ref_for_pid);
}

/**
 * @brief 重置跳跃子状态机，避免切走状态后残留跳跃阶段。
 */
void balance_Chassis::ResetJumpState() {
  jump_status = JUMP_NONE;
  jump_timer = 0;
  jump_length_ready_count_ = 0;
  jump_retract_hold_count_ = 0;
  jump_landing_ready_count_ = 0;
  jump_wheel_integral_l_ = 0.0f;
  jump_wheel_integral_r_ = 0.0f;
  jump_active_wheel_ref_l_ = 0.0f;
  jump_active_wheel_ref_r_ = 0.0f;
  jump_landing_speed_scale_ = 1.0f;
  jump_land_phi0_ref_l_ = 0.0f;
  jump_land_phi0_ref_r_ = 0.0f;
  jump_landing_brake_ref_l_ = 0.0f;
  jump_landing_brake_ref_r_ = 0.0f;
  jump_left_contact_count_ = jump_right_contact_count_ = 0U;
  jump_left_release_count_ = jump_right_release_count_ = 0U;
  jump_left_contact_ = jump_right_contact_ = false;
  jump_both_contact_latched_ = false;
  jump_landing_zero_cross_l_ = false;
  jump_landing_zero_cross_r_ = false;
  jump_pitch_integral_ = 0.0f;
  jump_roll_integral_ = 0.0f;
  jump_liftoff_seen_ = false;
  jump_state_ = false;
}

/**
 * @brief 统一处理状态切换时的进入/退出动作。
 * @param new_state 目标状态
 */
void balance_Chassis::ChangeState(RobotStatus new_state) {
  // 如果没有新状态就直接返回
  if (robot_status == new_state) {
    return;
  }
  const RobotStatus previous_state = robot_status;
  // 当从跳跃状态切换到其他状态时，会先清除跳跃状态的子状态
  if (robot_status == STATE_JUMPING) {
    ResetJumpState();
  }
  robot_status = new_state;
  fall_detect_count_ = 0;
  //
  if (new_state == STATE_RECOVERING) {
    const bool manual_dm_enable =
        previous_state == STATE_ESTOP && dm_estop_confirmed_ &&
        (dm_fault_latched_mask_ != 0U || dm_offline_latched_mask_ != 0U ||
         dm_disabled_latched_mask_ != 0U);
    if (!manual_dm_enable) {
      estop_reason_ = 0;
    }
    recover_request_latched_ = true;
    ResetRecoverState();
    if (manual_dm_enable) {
      // Exactly one Enable frame per motor on the confirmed ESTOP -> RECOVER
      // edge.  Fault history remains latched and no ClearError is sent.
      lf_joint_.Enable();
      lb_joint_.Enable();
      rf_joint_.Enable();
      rb_joint_.Enable();
    }
    recover_enable_pending_ = manual_dm_enable;
    dm_enable_wait_count_ = 0U;
    dm_estop_confirmed_ = false;
    roll_comp_.Clear();
    roll_force_cmd_ = 0.0f;
    roll_len_delta_cmd_ = 0.0f;
    DaoDiFlg_ = false; // 进入自起状态，清除倒地标志
    Z_Flag_ = false;
  } else if (new_state == STATE_ESTOP) {
    dm_estop_confirmed_ = false;
    dm_enable_wait_count_ = 0U;
    warming_counter_ = 0; // 进入急停/倒地状态，重置急停持续计数（全停零力矩）
    DaoDiFlg_ = true;     // 标记为倒地状态
  } else if (new_state == STATE_JUMPING) {
    jump_status = JUMP_COMPRESS;
    jump_timer = 0;
    jump_length_ready_count_ = 0;
    jump_retract_hold_count_ = 0;
    jump_landing_ready_count_ = 0;
    jump_wheel_integral_l_ = 0.0f;
    jump_wheel_integral_r_ = 0.0f;
    jump_wheel_speed_ref_l_ = 0.0f;
    jump_wheel_speed_ref_r_ = 0.0f;
    jump_active_wheel_ref_l_ = 0.0f;
    jump_active_wheel_ref_r_ = 0.0f;
    jump_landing_speed_scale_ = 1.0f;
    jump_land_phi0_ref_l_ = 0.0f;
    jump_land_phi0_ref_r_ = 0.0f;
    jump_landing_brake_ref_l_ = 0.0f;
    jump_landing_brake_ref_r_ = 0.0f;
    jump_left_contact_count_ = jump_right_contact_count_ = 0U;
    jump_left_release_count_ = jump_right_release_count_ = 0U;
    jump_left_contact_ = jump_right_contact_ = false;
    jump_both_contact_latched_ = false;
    jump_landing_zero_cross_l_ = false;
    jump_landing_zero_cross_r_ = false;
    jump_pitch_integral_ = 0.0f;
    jump_roll_integral_ = 0.0f;
    jump_liftoff_seen_ = false;
    jump_state_ = false;
  }
  if (new_state == STATE_NORMAL &&
      (previous_state == STATE_RECOVERING ||
       previous_state == STATE_JUMPING)) {
    // Start the high-gain normal leg controller from the measured capture
    // length, then ramp toward the operator command without a reference step.
    normal_handoff_active_ = true;
    normal_handoff_len_l_ = left_leg_.GetLegLen();
    normal_handoff_len_r_ = right_leg_.GetLegLen();
    left_leg_len_.Clear();
    right_leg_len_.Clear();
    roll_comp_.Clear();
    dist_m_ = 0.0f;
    dist_ = 0.0f;
    target_dist_ = 0.0f;
    roll_force_cmd_ = 0.0f;
    normal_pivot_roll_integral_ = 0.0f;
    roll_len_delta_cmd_ = 0.0f;
    heading_initialized_ = false;
    translation_command_active_ = false;
    yaw_command_active_ = false;
    normal_pivot_yaw_rate_ref_ = 0.0f;
    normal_pivot_yaw_torque_cmd_ = 0.0f;
    normal_pivot_center_trim_torque_ = 0.0f;
    normal_pivot_center_prev_speed_ = 0.0f;
    normal_zero_speed_trim_torque_ = 0.0f;
    normal_speed_ref_ = 0.0f;
    normal_wheel_center_speed_ = 0.0f;
    normal_wheel_diff_speed_ = 0.0f;
    normal_pivot_leg_sync_force_ = 0.0f;
    normal_pivot_leg_sync_integral_ = 0.0f;
    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_pitch_integral_ = 0.0f;
    normal_pivot_pitch_target_ = 0.0f;
    normal_handoff_leg_grace_count_ = kNormalHandoffLegGraceTicks;
    jump_normal_handoff_count_ =
        (previous_state == STATE_JUMPING) ? JUMP_NORMAL_HANDOFF_TICKS : 0U;
  } else if (new_state != STATE_NORMAL) {
    normal_handoff_active_ = false;
    heading_initialized_ = false;
    translation_command_active_ = false;
    yaw_command_active_ = false;
    normal_pivot_yaw_rate_ref_ = 0.0f;
    normal_pivot_yaw_torque_cmd_ = 0.0f;
    normal_pivot_center_trim_torque_ = 0.0f;
    normal_pivot_center_prev_speed_ = 0.0f;
    normal_zero_speed_trim_torque_ = 0.0f;
    normal_speed_ref_ = 0.0f;
    normal_wheel_center_speed_ = 0.0f;
    normal_wheel_diff_speed_ = 0.0f;
    normal_pivot_leg_sync_force_ = 0.0f;
    normal_pivot_leg_sync_integral_ = 0.0f;
    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_pitch_integral_ = 0.0f;
    normal_pivot_pitch_target_ = 0.0f;
    normal_handoff_leg_grace_count_ = 0;
    jump_normal_handoff_count_ = 0;
    normal_pivot_roll_integral_ = 0.0f;
  }
}

bool balance_Chassis::IsAutoFallTriggered() {
  const bool body_fall = (fabsf(INS.Pitch) > kNormalFallPitch) ||
                         (fabsf(INS.Roll) > kNormalFallRoll);

  // Immediately after recovery, NORMAL deliberately accepts a short leg and
  // a sizeable phi0 error while its regular controller raises the chassis.
  // Suppress only the leg-angle criterion during that hand-off; a genuinely
  // large body tilt still triggers recovery through body_fall.
  const bool handoff_leg_grace = normal_handoff_leg_grace_count_ > 0U;
  if (normal_handoff_leg_grace_count_ > 0U)
    normal_handoff_leg_grace_count_--;

  bool leg_angle_fall = false;
  if (!body_fall && !handoff_leg_grace) {
    const float left_phi0 = left_leg_.GetPhi0();
    const float right_phi0 = right_leg_.GetPhi0();
    const float left_err = fabsf(left_phi0 - 0.5f * PI);
    const float right_err = fabsf(right_phi0 - 0.5f * PI);
    const float leg_angle_err_raw =
        (left_err > right_err) ? left_err : right_err;
    const float phi0_accel_offset =
        leg_angle_accel_gain * atan2f(acc_, STANDARD_GRAVITY);
    float leg_angle_err_compensated =
        leg_angle_err_raw - fabsf(phi0_accel_offset);
    if (leg_angle_err_compensated < 0.0f) {
      leg_angle_err_compensated = 0.0f;
    }
    leg_angle_fall = (leg_angle_err_compensated > kNormalFallLegAngle);
  }

  if (body_fall || leg_angle_fall) {
    if (fall_detect_count_ < k_fall_confirm_ticks) {
      fall_detect_count_++;
    }
  } else {
    fall_detect_count_ = 0;
  }
  return (fall_detect_count_ >= k_fall_confirm_ticks);
}

bool balance_Chassis::IsRecoverComplete() {
  // Once the body is capturable and both legs have reached the verified
  // 0.18 m hand-off length, NORMAL is responsible for pulling the chassis off
  // an auxiliary roller.  Requiring a second absolute-yaw hold here can leave
  // recovery trapped indefinitely in that static triangular support pose.
  return recover_state_ && recover_leg_retracted_ &&
         (recover_balance_cnt_ >= k_recover_pitch_balance_hold);
}

/**
 * @brief 只负责状态转移，不直接修改速度和腿长目标。
 */
void balance_Chassis::UpdateStateMachine() {
  // 先判断是否急停，急停优先级最高
  const int status_flag = (int)sbus_rx_data.status_flag;
  const uint8_t jump_flag = (uint8_t)sbus_rx_data.jump_flag;

  if (status_flag != 1) {
    recover_request_latched_ = false;
  }

  // SBUS 失控保护（优先级最高）：超过 SBUS_LINK_TIMEOUT_MS 未收到有效遥控帧 →
  // 强制急停
  // SBUS失控保护：自起过程中忽略瞬断，防止振动导致断连打断自起
  if (SBUS_IsLinkLost()) {
    // A hard flip can corrupt a few consecutive SBUS frames. In recovery only,
    // tolerate a bounded additional gap after the driver's 100 ms timeout;
    // all other modes retain the immediate link-loss ESTOP behavior.
    if (robot_status == STATE_RECOVERING &&
        recover_sbus_lost_count_ < k_recover_sbus_loss_grace_ticks) {
      recover_sbus_lost_count_++;
    } else {
      last_jump_flag_ = jump_flag;
      estop_reason_ = 1;
      ChangeState(STATE_ESTOP);
      return;
    }
  } else {
    recover_sbus_lost_count_ = 0;
  }

  // Z键模拟：SA拨杆从急停档(0/3)切回非急停档(1/2)时触发（上升沿检测）
  if (Last_Z_Key_ && (status_flag == 1 || status_flag == 2)) {
    Z_Flag_ = true;
  }
  Last_Z_Key_ = (status_flag == 0 || status_flag == 3);

  if (status_flag == 3 || status_flag == 0) {
    last_jump_flag_ = jump_flag;
    if (robot_status == STATE_ESTOP &&
        (dm_fault_latched_mask_ != 0U || dm_offline_latched_mask_ != 0U ||
         dm_disabled_latched_mask_ != 0U)) {
      dm_estop_confirmed_ = true;
    }
    if (dm_fault_latched_mask_ == 0U && dm_offline_latched_mask_ == 0U &&
        dm_disabled_latched_mask_ == 0U) {
      estop_reason_ = 2;
    }
    ChangeState(STATE_ESTOP);
    return;
  }
  // 小板凳调试模式，不管什么状态都记录上一次的jump_flag
  if (robot_status == STATE_JOINT_DEBUG) {
    last_jump_flag_ = jump_flag;
    return;
  }
  // Diagnose and latch the original DM condition.  No ClearError/Enable is
  // issued here, so VOFA preserves the failure scene until the controller is
  // rebooted.
  const bool waiting_for_manual_enable =
      robot_status == STATE_RECOVERING && recover_enable_pending_;
  const uint8_t dm_fault_now =
      waiting_for_manual_enable ? 0U : GetDmFaultMaskNow();
  const uint8_t dm_offline_now =
      waiting_for_manual_enable ? 0U : GetDmOfflineMaskNow();
  const uint8_t dm_disabled_now =
      waiting_for_manual_enable ? 0U : GetDmDisabledMaskNow();
  if (dm_fault_now != 0U || dm_offline_now != 0U || dm_disabled_now != 0U) {
    const bool first_dm_event = dm_fault_latched_mask_ == 0U &&
                                dm_offline_latched_mask_ == 0U &&
                                dm_disabled_latched_mask_ == 0U;
    dm_fault_latched_mask_ |= dm_fault_now;
    dm_offline_latched_mask_ |= dm_offline_now;
    dm_disabled_latched_mask_ |= dm_disabled_now;
    if (first_dm_event) {
      estop_reason_ = (dm_fault_now != 0U) ? 4U
                      : (dm_offline_now != 0U) ? 5U
                                              : 6U;
    }
    if (robot_status != STATE_ESTOP) {
      ChangeState(STATE_ESTOP);
      return;
    }
  }
  // 如果机器人处于跳跃状态且没有急停打断，则必须跳跃状态结束之后才能且回到正常状态
  if (robot_status == STATE_JUMPING) {
    // 保持压缩蓄力，直到跳跃开关从 2 回到 1；上方的急停和失联保护
    // 仍然具有更高优先级。
    if (jump_status == JUMP_COMPRESS && last_jump_flag_ == 2 &&
        jump_flag == 1) {
      // Capture each motor's measured speed on the exact 2 -> 1 edge. During
      // extension the jump-only wheel loop holds these values unchanged.
      jump_wheel_speed_ref_l_ = JUMP_ASCEND_WHEEL_SPEED_CAPTURE_SCALE *
                                left_wheel.Get_Now_Omega();
      jump_wheel_speed_ref_r_ = JUMP_ASCEND_WHEEL_SPEED_CAPTURE_SCALE *
                                right_wheel.Get_Now_Omega();
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      jump_status = JUMP_ASCEND;
      jump_timer = 0;
      jump_length_ready_count_ = 0;
      jump_retract_hold_count_ = 0;
      jump_liftoff_seen_ = false;
    }
    last_jump_flag_ = jump_flag;
    if (jump_status == JUMP_NONE) {
      if (status_flag == 1) {
        ChangeState(STATE_RECOVERING);
      } else {
        ChangeState(STATE_NORMAL);
      }
    }
    return;
  }

  // 急停状态
  // 所有电机强制零输出，直到遥控器切出急停档。
  if (robot_status == STATE_ESTOP) {
    last_jump_flag_ = jump_flag;
    if (dm_fault_latched_mask_ != 0U || dm_offline_latched_mask_ != 0U ||
        dm_disabled_latched_mask_ != 0U) {
      if (dm_estop_confirmed_ && status_flag == 1 &&
          !recover_request_latched_) {
        ChangeState(STATE_RECOVERING);
      }
      return;
    }
    // SA 切出急停档：1 → 倒地自起，2 → 直接正常行驶
    if (status_flag == 1 && !recover_request_latched_) {
      ChangeState(STATE_RECOVERING);
      return;
    }
    if (status_flag == 2 && !recover_failed_) {
      ChangeState(STATE_NORMAL);
      return;
    }
    // SA 仍为 0/3，保持 ESTOP（全停零力矩）
    return;
  }

  // 自起状态：检查是否完成
  if (robot_status == STATE_RECOVERING) {
    last_jump_flag_ = jump_flag;
    // SA=2 is an operator-authorized hand-off to the normal controller. Only
    // accept it after the arbitrary-pose stages have reached the capturable
    // balance state and the body is upright; normal leg control can then raise
    // the remaining short support pose using its regular length command.
    if (status_flag == 2 && recover_sub_status_ == RECOVER_BALANCE &&
        recover_sub_timer_ >= k_recover_manual_handoff_hold &&
        fabsf(INS.Pitch) < k_recover_center_fallback_angle &&
        fabsf(INS.Roll) < k_recover_center_fallback_angle &&
        fabsf(INS.Gyro[0]) < k_recover_body_rate_ok &&
        fabsf(INS.Gyro[1]) < k_recover_body_rate_ok &&
        fabsf(vel_) < k_recover_balance_speed_limit &&
        left_leg_.GetLegLen() >= k_recover_joint_lift_release_len &&
        right_leg_.GetLegLen() >= k_recover_joint_lift_release_len) {
      ChangeState(STATE_NORMAL);
      return;
    }
    if (IsRecoverComplete()) {
      ChangeState(STATE_NORMAL);
    }
    return;
  }

  // ====== 以下仅从 STATE_NORMAL 进入 ======

  // 正常行驶中确认倒地后自动进入恢复流程；200 个控制周期的去抖可避免
  // 加减速或越障瞬态误触发。
  if (IsAutoFallTriggered()) {
    last_jump_flag_ = jump_flag;
    ChangeState(STATE_RECOVERING);
    return;
  }

  // SA=1 在正常模式下强制进入自起
  if (status_flag == 1 && !recover_request_latched_) {
    last_jump_flag_ = jump_flag;
    ChangeState(STATE_RECOVERING);
    return;
  }

  // 跳跃触发检测（仅 NORMAL 状态下允许）
  if (last_jump_flag_ == 1 && jump_flag == 2 && jump_status == JUMP_NONE) {
    jump_state_ = true;
  }
  last_jump_flag_ = jump_flag;

  if (jump_state_) {
    ChangeState(STATE_JUMPING);
  }
}

/**
 * @brief 根据当前状态刷新控制目标，不负责状态切换。
 */
void balance_Chassis::UpdateCommandByState() {
  if (robot_status == STATE_NORMAL) {
    SetLegLen();
    SetSpd();
    return;
  }

  // Compression remains inside the jump controller but keeps refreshing the
  // operator speed/yaw commands. Only its leg-length reference is forced to the
  // minimum in JumpCalc(). No NORMAL controller state or code path is reused.
  if (robot_status == STATE_JUMPING && jump_status == JUMP_COMPRESS) {
    SetSpd();
    return;
  }

  // Once extension starts, translation/yaw commands no longer alter the
  // references captured on the jump switch's 2 -> 1 edge.
  if (robot_status == STATE_JUMPING) {
    target_speed_ = 0.0f;
    target_w_rotation_ = 0.0f;
    target_dist_ = 0.0f;
    return;
  }

  if (robot_status == STATE_JOINT_DEBUG) {
    SetSpd();
    return;
  }

  target_speed_ = 0.0f;
  target_w_rotation_ = 0.0f;
  target_dist_ = 0.0f;
}

/**
 * @brief 更新顶层状态机
 * @param
 * @param
 * @param
 */
void balance_Chassis::UpdateChassisStatus() {
  // 旧接口保留，用于兼容历史调用点。
  UpdateStateMachine();
}
/**
 * @brief 执行当前状态的子控制器
 * @param
 * @param
 * @param
 */
void balance_Chassis::ChassissControl() {
  switch (robot_status) {
  case STATE_NORMAL: {
    NormalCalc();
    break;
  }
  case STATE_ESTOP: {
    // 参考山海机甲：倒地后分级阻尼/支撑
    WarmingMotorControl();
    break;
  }
  case STATE_RECOVERING: {
    RecoverCalc();
    break;
  }
  case STATE_JUMPING: {
    JumpCalc();
    break;
  }
  case STATE_JOINT_DEBUG: {
    JointDebugCalc();
    break;
  }
  default:
    break;
  }
}
/**
 * @brief 正常行驶状态的控制逻辑
 * @param
 * @param
 * @param
 */
void balance_Chassis::NormalCalc() {
  const float normal_pitch = INS.Pitch - kNormalPitchZeroOffset;
  const bool jump_landing_handoff = jump_normal_handoff_count_ > 0U;
  const bool normal_grounded = jump_landing_handoff || !GetOffGround();
  float jump_landing_wheel_scale = 1.0f;
  if (jump_landing_handoff) {
    const uint32_t elapsed =
        JUMP_NORMAL_HANDOFF_TICKS - jump_normal_handoff_count_;
    const float wheel_ramp = ClampRange(
        (float)elapsed / (float)JUMP_NORMAL_WHEEL_RAMP_TICKS, 0.0f, 1.0f);
    // Retain enough immediate balance/braking authority to stop the post-jump
    // forward run, then restore full NORMAL authority without a torque step.
    jump_landing_wheel_scale =
        JUMP_NORMAL_WHEEL_MIN_SCALE +
        (1.0f - JUMP_NORMAL_WHEEL_MIN_SCALE) * wheel_ramp;
    jump_normal_handoff_count_--;
  }
  float left_ref = target_len_;
  float right_ref = target_len_;
  float left_ff = 0.0f;
  float right_ff = 0.0f;
  if (normal_handoff_active_) {
    normal_handoff_len_l_ = SlewTowards(normal_handoff_len_l_, target_len_,
                                        k_normal_handoff_len_slew_per_tick);
    normal_handoff_len_r_ = SlewTowards(normal_handoff_len_r_, target_len_,
                                        k_normal_handoff_len_slew_per_tick);
    left_ref = normal_handoff_len_l_;
    right_ref = normal_handoff_len_r_;
    if (fabsf(normal_handoff_len_l_ - target_len_) <
            k_normal_handoff_len_slew_per_tick &&
        fabsf(normal_handoff_len_r_ - target_len_) <
            k_normal_handoff_len_slew_per_tick) {
      normal_handoff_active_ = false;
    }
  }

  const bool translation_idle =
      fabsf(target_speed_) <= kTranslationCommandDeadband;
  const bool yaw_turn = fabsf(target_w_rotation_) > kYawCommandDeadband;
  const bool pivot_turn = translation_idle && yaw_turn;

  // A NORMAL in-place turn uses one common length reference. In particular,
  // do not carry the two independent recovery hand-off references into a pivot:
  // equal references are the geometric prerequisite for zero body roll.
  if (pivot_turn) {
    const float pivot_leg_ref = 0.5f * (left_ref + right_ref);
    left_ref = pivot_leg_ref;
    right_ref = pivot_leg_ref;
  }

  // NORMAL gets its own second-stage command shaper.  The SBUS ramp is shared
  // by every state, so changing it would also change recovery behaviour.  A
  // bounded speed slope here reduces the pitch excursion caused by a sudden
  // translation command.  A pivot always requests exactly zero common speed.
  const float normal_speed_goal = translation_idle ? 0.0f : target_speed_;
  float normal_speed_step = kNormalSpeedRefAccel * 0.001f;
  if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
    normal_speed_step = kNormalSpeedRefAccel * controller_dt_;
  }
  if (fabsf(normal_speed_goal) < fabsf(normal_speed_ref_)) {
    normal_speed_step *= kNormalSpeedRefDecel / kNormalSpeedRefAccel;
  }
  normal_speed_ref_ =
      SlewTowards(normal_speed_ref_, normal_speed_goal, normal_speed_step);
  if (pivot_turn) {
    // An in-place turn has no translational ramp state.  Retaining the tail of
    // a previous drive command gives LQR a non-zero common-speed request and
    // moves the instantaneous rotation centre away from the axle midpoint.
    normal_speed_ref_ = 0.0f;
  }

  // The historical positive bias compensates drivetrain drift only while
  // translating.  Applying it at zero input or during a pivot creates a real
  // common wheel command and necessarily moves the instantaneous turn centre.
  const float normal_bias_scale =
      translation_idle
          ? 0.0f
          : ClampRange(fabsf(normal_speed_ref_) / kNormalSpeedBiasRampSpeed,
                       0.0f, 1.0f);
  const float normal_translation_bias = kNormalSpeedBias * normal_bias_scale;
  const float normal_common_speed_target =
      normal_speed_ref_ + normal_translation_bias;
  normal_target_left_wheel_speed_ = normal_common_speed_target;
  normal_target_right_wheel_speed_ = normal_common_speed_target;

  // Encoder-only wheel coordinates for NORMAL. The general chassis observer
  // also contains leg-angle and pitch-rate terms; feeding those terms back
  // while the legs shake creates a false translational velocity and closes a
  // positive oscillation loop. Keep that observer unchanged for other states.
  const float encoder_left_speed = -left_wheel.Get_Now_Omega() * k_wheel_radius;
  const float encoder_right_speed =
      right_wheel.Get_Now_Omega() * k_wheel_radius;
  const float encoder_center_speed =
      0.5f * (encoder_left_speed + encoder_right_speed);
  const float encoder_diff_speed =
      0.5f * (encoder_right_speed - encoder_left_speed);
  const float center_speed_lpf_alpha =
      pivot_turn ? kNormalPivotWheelSpeedLpfAlpha : kNormalWheelSpeedLpfAlpha;
  normal_wheel_center_speed_ +=
      center_speed_lpf_alpha *
      (encoder_center_speed - normal_wheel_center_speed_);
  normal_wheel_diff_speed_ += kNormalWheelSpeedLpfAlpha *
                              (encoder_diff_speed - normal_wheel_diff_speed_);

  // 正常模式下，状态函数自己决定腿长参考和腿长前馈。
  // 着地时使用支撑前馈；离地时切到缓冲腿长，并关闭前馈。
  if (normal_grounded) {
    const float mass_eff =
        0.5f * LEG_FF_BODY_MASS + LEG_FF_LEG_MASS_RATIO * LEG_FF_LEG_MASS;
    const float gravity_ff_left =
        mass_eff * STANDARD_GRAVITY * arm_cos_f32(left_leg_.GetTheta());
    const float gravity_ff_right =
        mass_eff * STANDARD_GRAVITY * arm_cos_f32(right_leg_.GetTheta());

    float inertial_ff = 0.0f;
    if (LEG_FF_WHEEL_TRACK > 1e-6f) {
      const float leg_len_mean =
          0.5f * (left_leg_.GetLegLen() * arm_cos_f32(left_leg_.GetTheta()) +
                  right_leg_.GetLegLen() * arm_cos_f32(right_leg_.GetTheta()));
      // A true pivot has zero axle-centre velocity, hence no translational
      // lateral-inertia feedforward.  The general observer can report a false
      // vel_ while the legs and pitch move; feeding that value into opposite
      // leg forces creates a roll error that grows with yaw rate.
      if (!pivot_turn) {
        inertial_ff = mass_eff * (leg_len_mean / (2.0f * LEG_FF_WHEEL_TRACK)) *
                      vel_ * INS.Gyro[2];
      }
    }

    // NORMAL-only low-bandwidth roll controller.  Do not numerically
    // differentiate the Euler angle at 1 kHz: the resulting D term was visible
    // on I10 as roughly -20..50 N noise.  The IMU gyro is the measured roll
    // rate, so use it directly for damping and filter the final axial-force
    // difference before it reaches VMC.
    const float roll_kp = pivot_turn ? kNormalPivotRollKp : kNormalRollKp;
    const float roll_kd = pivot_turn ? kNormalPivotRollKd : kNormalRollKd;
    const float roll_force_limit =
        pivot_turn ? kNormalPivotRollForceMax : kNormalRollOutMax;
    const float roll_ki = pivot_turn ? kNormalPivotRollKi : kNormalRollKi;
    const float roll_integral_limit =
        pivot_turn ? kNormalPivotRollIntegralMax : kNormalRollIntegralMax;
    const float roll_target = kNormalRollZeroOffset;
    const float roll_error = roll_target - INS.Roll;
    const float pivot_leg_diff_abs =
        fabsf(left_leg_.GetLegLen() - right_leg_.GetLegLen());
    const float pivot_roll_leg_scale =
        pivot_turn
            ? ClampRange((kPivotRollCutoffLegDiff - pivot_leg_diff_abs) /
                             (kPivotRollCutoffLegDiff - kPivotRollFullLegDiff),
                         0.0f, 1.0f)
            : 1.0f;
    if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
      if (!pivot_turn || pivot_roll_leg_scale > 0.0f) {
        normal_pivot_roll_integral_ = ClampAbs(
            normal_pivot_roll_integral_ +
                pivot_roll_leg_scale * roll_ki * roll_error * controller_dt_,
            roll_integral_limit);
      } else {
        // Equal leg length is a hard pivot constraint. Do not retain roll
        // integral energy while the measured length mismatch is out of range.
        normal_pivot_roll_integral_ = 0.0f;
      }
    } else {
      normal_pivot_roll_integral_ = 0.0f;
    }
    float normal_roll_raw = 0.0f;
    if (pivot_turn || fabsf(roll_error) > kRollDeadBand ||
        fabsf(INS.Gyro[0]) > 0.01f) {
      normal_roll_raw = pivot_roll_leg_scale *
                        ClampAbs(roll_kp * roll_error - roll_kd * INS.Gyro[0] +
                                     normal_pivot_roll_integral_,
                                 roll_force_limit);
    }
    roll_force_cmd_ = ClampAbs(roll_force_cmd_, roll_force_limit);
    const float roll_lpf_alpha =
        pivot_turn ? kNormalPivotRollForceLpfAlpha : kNormalRollForceLpfAlpha;
    if (pivot_turn && pivot_roll_leg_scale <= 0.0f) {
      roll_force_cmd_ = 0.0f;
    } else {
      roll_force_cmd_ += roll_lpf_alpha * (normal_roll_raw - roll_force_cmd_);
    }
    const float roll_out = roll_force_cmd_;

    // Use one physical roll direction throughout NORMAL. Hardware terrain data
    // showed that the old non-pivot branch used the opposite sign and therefore
    // enlarged roll instead of levelling the body. Keep the independently
    // derived lateral-inertia feedforward direction unchanged.
    left_ff = gravity_ff_left + roll_out - inertial_ff;
    right_ff = gravity_ff_right - roll_out + inertial_ff;

    // Preserve the operator's mean length command while allowing the two legs
    // to follow uneven terrain. Independent clamps use the available travel
    // near either mechanical limit instead of forcing equal target lengths.
    // Do not convert body roll into a geometric leg-length difference in
    // NORMAL. The measured test shows this path amplifies one loaded leg's
    // oscillation. Differential support force above already controls roll.
    roll_len_delta_cmd_ += kRollLenLpfAlpha * (0.0f - roll_len_delta_cmd_);
    // --- Roll 鍑犱綍鑵块暱琛ュ伩 ---
    // 鏃嬭浆鏃秗oll鍊炬枩
    // 鈫?宸垎鑵块暱浠ヤ繚鎸佹満浣撴按骞砛 roll>0
    // (鍙冲€? 鈫?宸﹁吙杩囬珮闇€鏀剁煭 鈫?left_ref -= delta, right_ref += delta
  } else {
    roll_force_cmd_ = 0.0f;
    normal_pivot_roll_integral_ = 0.0f;
    roll_len_delta_cmd_ = 0.0f;
    left_ref = OFF_GROUND_LEG_LENGTH;
    right_ref = OFF_GROUND_LEG_LENGTH;
  }

  // NORMAL-only axle-midpoint speed loop. It remains active during a pivot:
  // common wheel speed must stay at zero while the differential channel turns
  // the chassis. Otherwise a drivetrain/pitch bias is added to both wheels and
  // moves the instantaneous rotation centre outside the robot.
  if (translation_idle && !pivot_turn) {
    if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
      normal_zero_speed_trim_torque_ = ClampAbs(
          normal_zero_speed_trim_torque_ -
              // Idle-only trim direction verified on hardware; the pivot loop
              // below uses the opposite sign because it trims the turn drag.
              kNormalZeroSpeedKi * normal_wheel_center_speed_ * controller_dt_,
          kNormalZeroSpeedIntegralMax);
    }
  } else {
    // Do not store common-wheel integral energy during a turn.  It would
    // survive stick release as a backward command and move the pivot centre.
    normal_zero_speed_trim_torque_ = 0.0f;
  }

  // Cancel only the slow, persistent midpoint drift during an in-place turn.
  // NORMAL LQR retains its normal speed and pitch states; the bounded P term
  // below stops fast common-speed growth, while this integral removes the
  // remaining bias that leaves one wheel nearly stationary.  It is discarded
  // immediately outside a pivot.
  if (pivot_turn && controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
    // Pitch now has an independent common-hip PI-D loop below, so I2 no longer
    // needs a hard pitch gate. Integrate every pivot cycle to remove its own
    // steady common-speed error instead of remaining disabled near the gate.
    const float zero_cross_threshold_sq =
        kPivotCenterIntegratorDeadband * kPivotCenterIntegratorDeadband;
    if (normal_pivot_center_prev_speed_ * normal_wheel_center_speed_ <
        -zero_cross_threshold_sq) {
      // The measured I2 limit cycle crosses zero with a large stored trim.
      // Discard most of that old half-cycle energy before integrating the new
      // sign; retain a small fraction so genuine drivetrain bias is not lost.
      normal_pivot_center_trim_torque_ *= kPivotCenterZeroCrossRetention;
    }
    if (fabsf(normal_wheel_center_speed_) > kPivotCenterIntegratorDeadband) {
      normal_pivot_center_trim_torque_ = ClampAbs(
          normal_pivot_center_trim_torque_ +
              kPivotCenterSpeedKi * normal_wheel_center_speed_ * controller_dt_,
          kPivotCenterIntegralMax);
    }
    normal_pivot_center_prev_speed_ = normal_wheel_center_speed_;
  } else {
    normal_pivot_center_trim_torque_ = 0.0f;
    normal_pivot_center_prev_speed_ = 0.0f;
  }

  // Normal mode attitude/common-wheel controller.
  LQRCalc();
  SynthesizeMotion();

  if (normal_grounded && controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
    // Remove steady NORMAL pitch with the common hip channel instead of adding
    // a wheel-speed bias. The sign follows the fitted LQR hip/pitch
    // coefficient: positive measured pitch requires positive common hip torque.
    normal_pivot_pitch_integral_ =
        ClampAbs(normal_pivot_pitch_integral_ +
                     kNormalPivotHipPitchKi * normal_pitch * controller_dt_,
                 kNormalPivotHipPitchIntegralMax);
    const float normal_pitch_hip_torque = ClampAbs(
        kNormalPivotHipPitchKp * normal_pitch +
            kNormalPivotHipPitchKd * INS.Gyro[1] + normal_pivot_pitch_integral_,
        kNormalPivotHipPitchTorqueMax);
    left_leg_T_ = ClampAbs(left_leg_T_ + normal_pitch_hip_torque, 40.0f);
    right_leg_T_ = ClampAbs(right_leg_T_ + normal_pitch_hip_torque, 40.0f);
  } else {
    normal_pivot_pitch_integral_ = 0.0f;
  }

  // With both sticks centred, a large left/right hip-torque difference twists
  // the two legs in opposite directions.  That motion excites yaw, roll and
  // the wheel-speed observer even though no yaw was requested.  Keep the
  // common pitch-balancing torque, but bound only its differential component.
  // Differential axial force above remains responsible for body roll.
  if (translation_idle && !yaw_turn && normal_grounded) {
    const float leg_torque_common = 0.5f * (left_leg_T_ + right_leg_T_);
    const float leg_torque_diff = ClampAbs(0.5f * (right_leg_T_ - left_leg_T_),
                                           kNormalIdleLegDiffTorqueMax);
    left_leg_T_ = leg_torque_common - leg_torque_diff;
    right_leg_T_ = leg_torque_common + leg_torque_diff;
  }

  // Recompose wheel output as common torque (pitch + midpoint) and
  // differential torque (yaw).  The two coordinates are controlled
  // independently during a pivot: common wheel speed is held at exactly zero,
  // while differential wheel speed produces the requested yaw rate.
  if (pivot_turn) {
    float wheel_common = 0.5f * (l_wheel_T_ + r_wheel_T_);

    // Convert commanded body yaw rate to differential wheel speed, then use
    // measured IMU yaw rate as a small outer-loop correction.  Clamp the
    // corrected target to the commanded direction: an overspeed event asks the
    // inner loop to brake toward zero, never to reverse the requested turn.
    const float geometric_diff_speed =
        0.5f * LEG_FF_WHEEL_TRACK * target_w_rotation_;
    float target_diff_speed =
        geometric_diff_speed +
        kPivotYawRateToDiffSpeedKp * (target_w_rotation_ - INS.YawSpeed);
    const float max_diff_speed = 0.5f * LEG_FF_WHEEL_TRACK * YAW_SPEED_MAX;
    if (target_w_rotation_ > 0.0f) {
      target_diff_speed = ClampRange(target_diff_speed, 0.0f, max_diff_speed);
    } else {
      target_diff_speed = ClampRange(target_diff_speed, -max_diff_speed, 0.0f);
    }
    normal_target_left_wheel_speed_ = -target_diff_speed;
    normal_target_right_wheel_speed_ = target_diff_speed;

    // Zero centre speed is exactly equivalent to equal-magnitude,
    // opposite-sign wheel speeds.  Preserve the NORMAL LQR common/pitch output
    // and add a bounded midpoint P term plus the slow pivot trim above.
    const float center_speed_torque =
        ClampAbs(kPivotCenterSpeedTorqueSign * kPivotCenterSpeedKp *
                         (0.0f - normal_wheel_center_speed_) +
                     normal_pivot_center_trim_torque_,
                 kPivotCenterSpeedTorqueMax);
    wheel_common += center_speed_torque;
    normal_pivot_left_wheel_corr_ = center_speed_torque;

    const float diff_speed_error = target_diff_speed - normal_wheel_diff_speed_;
    float wheel_diff = ClampAbs(kNormalDiffSpeedTorqueSign *
                                    kPivotWheelSpeedKp * diff_speed_error,
                                kPivotYawTorqueMax);

    // Pitch and axle-centre speed are stability states, so preserve the common
    // channel first. The old differential-first allocation left too little
    // common torque at high yaw command: pitch stayed biased and I2 returned to
    // zero only after a long integral transient. Reserving at least 2 N.m for
    // yaw still keeps the chassis turning while common control remains active.
    wheel_common = ClampAbs(wheel_common, kNormalPivotCommonTorqueLimit);
    const float wheel_diff_limit =
        kNormalWheelTorqueLimit - fabsf(wheel_common);
    wheel_diff = ClampAbs(wheel_diff, wheel_diff_limit);
    l_wheel_T_ = wheel_common - wheel_diff;
    r_wheel_T_ = wheel_common + wheel_diff;

    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_yaw_torque_cmd_ = wheel_diff;
  } else if (false && yaw_turn) {
    // Per-wheel pivot speed control: each wheel tracks its own target so the
    // axle-midpoint average returns to zero when both wheels track.
    const float raw_target_diff_speed =
        0.5f * LEG_FF_WHEEL_TRACK * target_w_rotation_;
    const float yaw_command_ratio =
        ClampRange(fabsf(target_w_rotation_) / kNormalYawRateScale, 0.0f, 1.0f);
    const float high_speed_diff_gain = 1.0f + kPivotHighSpeedDiffExtraGain *
                                                  yaw_command_ratio *
                                                  yaw_command_ratio;
    const float base_target_diff_speed =
        raw_target_diff_speed * high_speed_diff_gain;
    const float pivot_common_speed_target =
        pivot_turn ? 0.0f : normal_common_speed_target;
    const float target_left_speed =
        pivot_common_speed_target - base_target_diff_speed;
    const float target_right_speed =
        pivot_common_speed_target + base_target_diff_speed;
    normal_target_left_wheel_speed_ = target_left_speed;
    normal_target_right_wheel_speed_ = target_right_speed;

    const float actual_left_speed =
        normal_wheel_center_speed_ - normal_wheel_diff_speed_;
    const float actual_right_speed =
        normal_wheel_center_speed_ + normal_wheel_diff_speed_;

    // Decouple common and differential authority: the common channel must
    // exceed the fitted LQR speed gain to hold I2 at zero, while the
    // differential channel keeps the gain that tracks the yaw target.
    const float common_speed_error =
        pivot_common_speed_target - normal_wheel_center_speed_;
    const float diff_speed_error =
        base_target_diff_speed - normal_wheel_diff_speed_;
    const float common_corr = ClampAbs(kPivotCommonSpeedKp * common_speed_error,
                                       kPivotCommonTorqueMax);
    const float diff_corr =
        ClampAbs(kPivotDiffSpeedKp * diff_speed_error, kPivotDiffTorqueMax);
    const float left_corr =
        ClampAbs(common_corr - diff_corr, kNormalWheelTorqueLimit);
    const float right_corr =
        ClampAbs(common_corr + diff_corr, kNormalWheelTorqueLimit);

    l_wheel_T_ = ClampAbs(l_wheel_T_ + left_corr, kNormalWheelTorqueLimit);
    r_wheel_T_ = ClampAbs(r_wheel_T_ + right_corr, kNormalWheelTorqueLimit);
    normal_pivot_left_wheel_corr_ = left_corr;
    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_pitch_integral_ = 0.0f;
    normal_pivot_yaw_torque_cmd_ = 0.0f;
  } else {
    float wheel_common = 0.5f * (l_wheel_T_ + r_wheel_T_);
    float wheel_diff = 0.5f * (r_wheel_T_ - l_wheel_T_);

    float center_loop_scale =
        1.0f - fabsf(INS.Pitch) / kNormalCenterLoopPitchFade;
    // During a pivot, body-pitch recovery has priority over holding the axle
    // midpoint at exactly zero speed.  Forcing both constraints while pitch is
    // displaced makes the midpoint loop cancel the wheel motion required by
    // the balance controller.  Once pitch returns, this scale rises to one and
    // the rotation centre is pulled back to the axle midpoint.
    // A pivot must retain midpoint-speed authority even while pitch is being
    // corrected.  Fading this loop to zero allowed the LQR common torque to
    // stop one wheel, moving the rotation centre onto that wheel.
    const float center_loop_min_scale =
        pivot_turn ? kPivotCenterLoopMinScale : kNormalCenterLoopMinScale;
    center_loop_scale =
        ClampRange(center_loop_scale, center_loop_min_scale, 1.0f);
    // Hold axle-midpoint speed at its target both while standing and pivoting.
    // Excluding pivot_turn here left the common wheel velocity uncontrolled,
    // so one wheel stopped and the chassis rotated about that wheel.
    if (translation_idle) {
      const float center_speed_kp =
          pivot_turn ? kPivotCenterSpeedKp : kNormalZeroSpeedKp;
      const float center_speed_torque_sign = pivot_turn
                                                 ? kPivotCenterSpeedTorqueSign
                                                 : kNormalCenterSpeedTorqueSign;
      const float center_speed_torque_max =
          pivot_turn ? kPivotCenterSpeedTorqueMax : kNormalZeroSpeedTorqueMax;
      // A centred translation stick means an exact zero axle-midpoint target.
      // In particular, never reuse the translation drift bias during a pivot.
      // During a pivot the turn drags the axle backward; hold a small forward
      // axle-midpoint target so the wheel pair stays symmetric around zero.
      float center_speed_target = 0.0f;
      if (pivot_turn) {
        const float yaw_ratio = ClampRange(
            fabsf(target_w_rotation_) / kNormalYawRateScale, 0.0f, 1.0f);
        center_speed_target =
            kPivotYawBiasCubicA * yaw_ratio * yaw_ratio * yaw_ratio +
            kPivotYawBiasCubicB * yaw_ratio * yaw_ratio +
            kPivotYawBiasCubicC * yaw_ratio + kPivotYawBiasCubicD;
      }
      const float center_speed_torque =
          center_loop_scale *
          ClampAbs(center_speed_torque_sign * center_speed_kp *
                           (center_speed_target - normal_wheel_center_speed_) +
                       (pivot_turn ? normal_pivot_center_trim_torque_
                                   : normal_zero_speed_trim_torque_),
                   center_speed_torque_max);
      // I11: actual common-wheel correction during a NORMAL pivot.  This lets
      // the VOFA trace distinguish an inactive centre loop from a correction
      // that is being opposed by the LQR common torque.
      normal_pivot_left_wheel_corr_ = pivot_turn ? center_speed_torque : 0.0f;
      wheel_common += center_speed_torque;
    }

    if (yaw_turn) {
      // Orthogonal pivot control: the common loop above holds axle-centre speed
      // at zero, while this loop controls only differential wheel speed. This
      // avoids two independent wheel PIs generating an unwanted common torque.
      // The operator yaw request is already a physical yaw-rate command. Do
      // not add a second IMU yaw-rate loop here: its sign/phase is different
      // from the encoder loop and the two loops previously fought until torque
      // saturation.  Only the high-speed end receives extra differential
      // authority, because the common positive bias otherwise drives both
      // wheels in the same direction and produces a large-radius circle.
      const float raw_target_diff_speed =
          0.5f * LEG_FF_WHEEL_TRACK * target_w_rotation_;
      const float yaw_command_ratio = ClampRange(
          fabsf(target_w_rotation_) / kNormalYawRateScale, 0.0f, 1.0f);
      const float high_speed_diff_gain = 1.0f + kPivotHighSpeedDiffExtraGain *
                                                    yaw_command_ratio *
                                                    yaw_command_ratio;
      const float base_target_diff_speed =
          raw_target_diff_speed * high_speed_diff_gain;
      // Equal and opposite wheel targets put the kinematic turn centre at the
      // axle midpoint.  Do not add a same-sign wheel-speed bias here.
      const float pivot_common_speed_target =
          pivot_turn ? 0.0f : normal_common_speed_target;
      const float target_left_speed =
          pivot_common_speed_target - base_target_diff_speed;
      const float target_right_speed =
          pivot_common_speed_target + base_target_diff_speed;
      normal_target_left_wheel_speed_ = target_left_speed;
      normal_target_right_wheel_speed_ = target_right_speed;
      // Keep the original differential target unchanged.  Folding the biased
      // targets back into a new differential target mostly slowed the other
      // wheel instead of accelerating the selected stalled wheel.
      const float target_diff_speed = base_target_diff_speed;
      const float diff_speed_error =
          target_diff_speed - normal_wheel_diff_speed_;

      // Do not let yaw consume wheel authority while the body or either leg is
      // leaving the balanced geometry.  This is a transient safety scheduler,
      // not a maximum-speed reduction: at small pitch and centred legs the
      // scale is exactly one.
      const float pitch_yaw_scale =
          ClampRange((kPivotYawCutoffPitch - fabsf(INS.Pitch)) /
                         (kPivotYawCutoffPitch - kPivotYawFullPitch),
                     0.0f, 1.0f);
      const float left_relative_angle = fabsf(left_leg_.GetTheta() - INS.Pitch);
      const float right_relative_angle =
          fabsf(right_leg_.GetTheta() - INS.Pitch);
      const float max_relative_angle =
          left_relative_angle > right_relative_angle ? left_relative_angle
                                                     : right_relative_angle;
      const float leg_yaw_scale =
          ClampRange((kPivotYawCutoffLegAngle - max_relative_angle) /
                         (kPivotYawCutoffLegAngle - kPivotYawFullLegAngle),
                     0.0f, 1.0f);
      // Keep full yaw authority even while the axle midpoint is off zero: the
      // forward-commanded wheel needs the extra differential torque to leave
      // its near-stall state, otherwise the whole-cycle average drifts back.
      const float center_speed_yaw_scale =
          pivot_turn
              ? ClampRange((kPivotFullCenterSpeed -
                            fabsf(normal_wheel_center_speed_)) /
                               (kPivotFullCenterSpeed - kPivotStartCenterSpeed),
                           1.0f, 1.0f)
              : 1.0f;
      const float yaw_balance_scale =
          pitch_yaw_scale * leg_yaw_scale * center_speed_yaw_scale;

      // Explicitly cancel every yaw integral. The wheel-speed P loop is the
      // only NORMAL yaw actuator, both during a pivot and while translating.
      normal_pivot_left_speed_integral_ = 0.0f;
      normal_pivot_right_speed_integral_ = 0.0f;

      const float speed_diff_torque = ClampAbs(
          kNormalDiffSpeedTorqueSign * (kPivotWheelSpeedKp * diff_speed_error) *
              yaw_balance_scale,
          kPivotYawTorqueMax);

      // Pitch remains a common wheel command and does not reduce either wheel's
      // speed target or the maximum differential turn command.
      // Keep pitch/common torque exclusively from the NORMAL LQR plus the
      // midpoint-speed loop above.

      normal_pivot_yaw_torque_cmd_ +=
          kPivotYawTorqueLpfAlpha *
          (speed_diff_torque - normal_pivot_yaw_torque_cmd_);
      wheel_diff = normal_pivot_yaw_torque_cmd_;

      // Do not add a second left/right speed P loop here.  The differential
      // controller above already tracks the target wheel-speed difference;
      // adding independent wheel P corrections created two high-gain loops on
      // the same motors and drove the wheel feedback into the large
      // oscillation seen during yaw commands.
    } else {
      // Centred yaw stick means zero differential torque immediately.  A
      // filtered tail here contaminates the following translation command.
      normal_pivot_yaw_torque_cmd_ = 0.0f;
      normal_pivot_left_speed_integral_ = 0.0f;
      normal_pivot_right_speed_integral_ = 0.0f;
      wheel_diff = 0.0f;
    }

    // Preserve the common/differential decomposition at saturation.  Giving
    // the differential command first priority keeps the wheel pair opposed;
    // independently clipping the two sides changes their mean and shifts the
    // turn centre whenever only one side saturates.
    if (pivot_turn || yaw_turn) {
      wheel_diff = ClampAbs(wheel_diff, kNormalWheelTorqueLimit);
      const float pivot_common_limit =
          kNormalWheelTorqueLimit - fabsf(wheel_diff);
      wheel_common = ClampAbs(wheel_common, pivot_common_limit);
      l_wheel_T_ = wheel_common - wheel_diff;
      r_wheel_T_ = wheel_common + wheel_diff;
    } else {
      wheel_common = ClampAbs(wheel_common, kNormalWheelTorqueLimit);
      float diff_limit = kNormalWheelTorqueLimit - fabsf(wheel_common);
      if (diff_limit < 0.0f)
        diff_limit = 0.0f;
      wheel_diff = ClampAbs(wheel_diff, diff_limit);
      l_wheel_T_ = wheel_common - wheel_diff;
      r_wheel_T_ = wheel_common + wheel_diff;
    }
  }
  // Leg-length control is parameterised; this function is reached only through
  // the STATE_NORMAL branch in ChassissControl().
  active_left_leg_ref_ = left_ref;
  active_right_leg_ref_ = right_ref;
  LegLenCalc(left_ref, right_ref, left_ff, right_ff);

  // Synchronise measured leg lengths throughout NORMAL.  Equal references alone
  // cannot remove a persistent left/right load or mechanism bias; this bounded
  // differential loop closes that missing state without touching hip torques.
  if (normal_grounded) {
    // Use full differential length authority whenever a NORMAL pivot is active.
    // Scheduling it down at low/medium yaw left a visible length mismatch even
    // though the requested references were equal.
    const float pivot_leg_sync_blend = pivot_turn ? 1.0f : 0.0f;
    const float leg_sync_kp =
        kNormalLegSyncKp +
        (kPivotLegSyncKp - kNormalLegSyncKp) * pivot_leg_sync_blend;
    const float leg_sync_kd =
        kNormalLegSyncKd +
        (kPivotLegSyncKd - kNormalLegSyncKd) * pivot_leg_sync_blend;
    const float leg_sync_force_max =
        kNormalLegSyncForceMax +
        (kPivotLegSyncForceMax - kNormalLegSyncForceMax) * pivot_leg_sync_blend;
    const float leg_length_diff =
        left_leg_.GetLegLen() - right_leg_.GetLegLen();
    if (pivot_turn && controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
      normal_pivot_leg_sync_integral_ =
          ClampAbs(normal_pivot_leg_sync_integral_ +
                       kPivotLegSyncKi * leg_length_diff * controller_dt_,
                   kPivotLegSyncIntegralMax);
    } else {
      normal_pivot_leg_sync_integral_ = 0.0f;
    }
    const float leg_sync_raw = ClampAbs(
        leg_sync_kp * leg_length_diff +
            leg_sync_kd * (left_leg_.GetLegSpeed() - right_leg_.GetLegSpeed()) +
            normal_pivot_leg_sync_integral_,
        leg_sync_force_max);
    normal_pivot_leg_sync_force_ +=
        kPivotLegSyncLpfAlpha * (leg_sync_raw - normal_pivot_leg_sync_force_);
    left_leg_F_ -= normal_pivot_leg_sync_force_;
    right_leg_F_ += normal_pivot_leg_sync_force_;
  } else {
    normal_pivot_leg_sync_force_ = 0.0f;
    normal_pivot_leg_sync_integral_ = 0.0f;
  }
  if (jump_landing_handoff) {
    l_wheel_T_ *= jump_landing_wheel_scale;
    r_wheel_T_ *= jump_landing_wheel_scale;
  }
  TorCalc();
}

uint8_t balance_Chassis::GetDmFaultMaskNow() const {
  uint8_t mask = 0U;
  if (lf_joint_.HasDriverFault())
    mask |= 0x01U;
  if (lb_joint_.HasDriverFault())
    mask |= 0x02U;
  if (rf_joint_.HasDriverFault())
    mask |= 0x04U;
  if (rb_joint_.HasDriverFault())
    mask |= 0x08U;
  return mask;
}

uint8_t balance_Chassis::GetDmOfflineMaskNow() const {
  uint8_t mask = 0U;
  if (!lf_joint_.IsFeedbackOnline())
    mask |= 0x01U;
  if (!lb_joint_.IsFeedbackOnline())
    mask |= 0x02U;
  if (!rf_joint_.IsFeedbackOnline())
    mask |= 0x04U;
  if (!rb_joint_.IsFeedbackOnline())
    mask |= 0x08U;
  return mask;
}

uint8_t balance_Chassis::GetDmDisabledMaskNow() const {
  uint8_t mask = 0U;
  if (lf_joint_.HasDriverDisabled())
    mask |= 0x01U;
  if (lb_joint_.HasDriverDisabled())
    mask |= 0x02U;
  if (rf_joint_.HasDriverDisabled())
    mask |= 0x04U;
  if (rb_joint_.HasDriverDisabled())
    mask |= 0x08U;
  return mask;
}

/**
 * @brief 倒地自起控制（参考山海机甲开源：Chassis_StandFromGround）
 *
 * 两阶段流程：
 *   阶段1（扫腿+收腿）：轮组自由（零力矩），髋关节 phi0 PID
 * 沿长路径把腿绕到身后并 保持伸展，接近 90° 目标后再收腿到
 * 0.25；收腿到位判定通过角度+腿长+速度稳定。 阶段2（平衡）：轮组 LQR 平衡 +
 * 重力前馈 + Yaw 差动轮矩对齐； 平衡检测通过后，状态机切入 STATE_NORMAL。
 * @param
 * @param
 * @param
 */
#if 0
// Historical two-stage draft retained temporarily for parameter comparison.
void balance_Chassis::RecoverCalcLegacyDisabled() {
  // --- 电机使能等待 ---
  if (recover_enable_pending_) {
    const bool all_enabled =
        (lf_joint_.Get_Status() == Motor_DM_Status_ENABLE) &&
        (lb_joint_.Get_Status() == Motor_DM_Status_ENABLE) &&
        (rf_joint_.Get_Status() == Motor_DM_Status_ENABLE) &&
        (rb_joint_.Get_Status() == Motor_DM_Status_ENABLE);
    if (!all_enabled) {
      l_wheel_T_ = 0.0f;
      r_wheel_T_ = 0.0f;
      left_leg_F_ = 0.0f;
      right_leg_F_ = 0.0f;
      left_leg_T_ = 0.0f;
      right_leg_T_ = 0.0f;
      TorCalc();
      return;
    }
    recover_enable_pending_ = false;
  }

  // --- 初始化 ---
  if (recover_timer == 0U) {
    recover_state_ = false;
    recover_leg_retracted_ = false;
    recover_balance_cnt_ = 0;
    recover_yaw_cnt_ = 0;
    shoutui_count = 0;
  }
  if (recover_timer < k_recover_timeout) {
    recover_timer++;
  }

  // --- 已站起后重新进入 RECOVERING：直接进入平衡阶段，避免再次扫腿 ---
  const bool already_upright =
      (fabsf(INS.Pitch) < k_recover_pitch_balance_th) &&
      (fabsf(left_leg_.GetPhi0() - k_recover_phi0_target) < k_phi0_ok) &&
      (fabsf(right_leg_.GetPhi0() - k_recover_phi0_target) < k_phi0_ok);
  if (already_upright) {
    recover_leg_retracted_ = true;
    recover_dynamic_len_ref_ = recover_final_target_len;
  }

  if (!recover_leg_retracted_) {
    // ================= 阶段1：扫腿 + 收腿 =================
    // 轮组自由（不参与），等待腿部把机身撑起
    l_wheel_T_ = 0.0f;
    r_wheel_T_ = 0.0f;
    dist_ = 0.0f;
    target_dist_ = 0.0f;

    // 根据机器人姿态判断长/短路径：
    // 面朝下(|Pitch|>45°)且腿在前(phi0<90°)→长路径绕后，避免腿扫向地面
    // 仰躺/侧躺(|Roll|>|Pitch|)→短路径，腿可直接扫到身后
    const bool face_down = fabsf(INS.Pitch) > (PI / 4.0f);
    const bool use_long_path_l =
        face_down && (left_leg_.GetPhi0() < k_recover_phi0_target);
    const bool use_long_path_r =
        face_down && (right_leg_.GetPhi0() < k_recover_phi0_target);

    // 仰躺：目标210°（腿斜向推地→水平分力滚动机身）
    // 面朝下：目标252°（腿向下撑地）
    const bool back_down = fabsf(INS.Roll) > (PI * 0.6f);
    // 仰躺：目标300°（不可及→误差永远大→积分堆满→全力推地翻转）
    // 面朝下：目标252°（腿向下撑地）
    const float sweep_phi0_target =
        back_down ? (4.71f) : k_recover_phi0_target; // 270° vs 252°
    const float sweep_leg_len =
        back_down ? 0.38f : recover_tmp_target_len; // 0.38 vs 0.35

    // 一旦越过目标角即锁存（只置位不复位）
    if (!recover_passed_pi_l_ &&
        left_leg_.GetPhi0() >= sweep_phi0_target) {
      recover_passed_pi_l_ = true;
    }
    if (!recover_passed_pi_r_ &&
        right_leg_.GetPhi0() >= sweep_phi0_target) {
      recover_passed_pi_r_ = true;
    }

    float leg_len_ref_l = recover_final_target_len;
    float leg_len_ref_r = recover_final_target_len;
    ConfigureRecoverLegControl(left_leg_.GetPhi0(), use_long_path_l,
                               recover_passed_pi_l_, left_leg_phi0,
                               leg_len_ref_l);
    ConfigureRecoverLegControl(right_leg_.GetPhi0(), use_long_path_r,
                               recover_passed_pi_r_, right_leg_phi0,
                               leg_len_ref_r);

    if (back_down) {
      // 仰躺翻转：恒定20Nm全力推地
      const float kSweepTorque = 20.0f;
      const float kSpeedDamping = 15.0f;
      left_leg_T_ =
          kSweepTorque - kSpeedDamping * left_leg_.GetPhi0Speed();
      right_leg_T_ =
          kSweepTorque - kSpeedDamping * right_leg_.GetPhi0Speed();
      left_leg_F_ = 0.0f;
      right_leg_F_ = 0.0f;
    } else {
      // 翻过来后：山海机甲原版 phi0 PID + 腿长PID 精准站立
      left_leg_phi0.SetMeasure(left_leg_.GetPhi0());
      right_leg_phi0.SetMeasure(right_leg_.GetPhi0());
      left_leg_phi0.SetRef(sweep_phi0_target);
      right_leg_phi0.SetRef(sweep_phi0_target);
      left_leg_T_ = left_leg_phi0.Calculate();
      right_leg_T_ = right_leg_phi0.Calculate();
      LegLenCalc(recover_final_target_len, recover_final_target_len, 0.0f, 0.0f);
    }

    const float err_l = fabsf(sweep_phi0_target - left_leg_.GetPhi0());
    const float err_r = fabsf(sweep_phi0_target - right_leg_.GetPhi0());
    recover_dynamic_len_ref_ = left_leg_.GetLegLen();
    TorCalc();

    // --- 收腿到位判定 ---
    const bool push_right = (INS.Roll > 0.0f);
    float len_target_l = back_down ? (push_right ? MIN_LEG_LENGTH : 0.30f)
                                   : recover_final_target_len;
    float len_target_r = back_down ? (push_right ? 0.30f : MIN_LEG_LENGTH)
                                   : recover_final_target_len;
    const bool len_ok_l =
        fabsf(left_leg_.GetLegLen() - len_target_l) < k_shoutui_length_th;
    const bool len_ok_r =
        fabsf(right_leg_.GetLegLen() - len_target_r) < k_shoutui_length_th;
    const bool angle_ok_l = (err_l < k_phi0_ok);
    const bool angle_ok_r = (err_r < k_phi0_ok);
    const bool speed_ok =
        (fabsf(left_leg_.GetPhi0Speed()) < k_phi0speed_ok) &&
        (fabsf(left_leg_.GetLegSpeed()) < k_shoutui_speed_th) &&
        (fabsf(right_leg_.GetPhi0Speed()) < k_phi0speed_ok) &&
        (fabsf(right_leg_.GetLegSpeed()) < k_shoutui_speed_th);

    if (len_ok_l && len_ok_r && angle_ok_l && angle_ok_r && speed_ok) {
      if (shoutui_count < k_shoutui_ok_hold) {
        shoutui_count++;
      }
    } else {
      shoutui_count = 0;
    }

    if (shoutui_count >= k_shoutui_ok_hold) {
      recover_leg_retracted_ = true;
      shoutui_count = 0;
      recover_dynamic_len_ref_ = recover_final_target_len;
    }
  } else {
    // ================= 阶段2：LQR 平衡 + Yaw 对齐 =================
    LQRCalc();

    // 支撑前馈 + 保持腿长
    const float base_ff_left =
        k_gravity_comp * arm_cos_f32(left_leg_.GetTheta());
    const float base_ff_right =
        k_gravity_comp * arm_cos_f32(right_leg_.GetTheta());
    LegLenCalc(recover_final_target_len, recover_final_target_len, base_ff_left,
               base_ff_right);
    SynthesizeMotion();

    // 刚进阶段2时重锁yaw，避免翻转过程的yaw漂移被当作误差修正
    if (recover_balance_cnt_ == 0 && recover_yaw_cnt_ == 0) {
      recover_yaw_target_ = INS.Yaw;
    }
    // Yaw 对齐差动轮矩（参考山海机甲 Chassis_StandFromGround）
    float yaw_err = INS.Yaw - recover_yaw_target_;
    if (yaw_err > PI) {
      yaw_err -= 2.0f * PI;
    } else if (yaw_err < -PI) {
      yaw_err += 2.0f * PI;
    }
    const float yaw_w =
        ClampAbs(k_recover_yaw_k1 * yaw_err, k_recover_yaw_w_limit);
    const float yaw_tau = k_recover_yaw_k2 * (INS.YawSpeed + yaw_w);
    // 注意：SetMotorTor 对左轮取反(-l_wheel_T_)，因此这里左右都加同号，
    // 经过取反后才能产生真正的差动：左轮后退、右轮前进 → CCW旋转修正Yaw
    l_wheel_T_ += 0.5f * yaw_tau;
    r_wheel_T_ += 0.5f * yaw_tau;

    TorCalc();
    recover_state_ = true;

    // --- 平衡检测：Pitch + 腿倾角达标后，再对齐 Yaw ---
    if (fabsf(INS.Pitch) < k_recover_pitch_balance_th &&
        fabsf(left_leg_.GetTheta()) < k_theta_ok_rad &&
        fabsf(right_leg_.GetTheta()) < k_theta_ok_rad) {
      if (recover_balance_cnt_ < k_recover_pitch_balance_hold) {
        recover_balance_cnt_++;
      }
    } else {
      recover_balance_cnt_ = 0;
    }

    if (recover_balance_cnt_ >= k_recover_pitch_balance_hold) {
      yaw_err = INS.Yaw - recover_yaw_target_;
      if (yaw_err > PI) {
        yaw_err -= 2.0f * PI;
      } else if (yaw_err < -PI) {
        yaw_err += 2.0f * PI;
      }
      if (fabsf(yaw_err) < k_yaw_ok_rad) {
        if (recover_yaw_cnt_ < k_recover_pitch_yaw_hold) {
          recover_yaw_cnt_++;
        }
      } else {
        recover_yaw_cnt_ = 0;
      }
    }
  }

  // --- 超时保护 ---
  if (recover_timer >= k_recover_timeout) {
    // 超时则重置，重新开始
    recover_timer = 0;
    shoutui_count = 0;
    recover_leg_retracted_ = false;
    recover_balance_cnt_ = 0;
    recover_yaw_cnt_ = 0;
  }
}
#endif

void balance_Chassis::RecoverCalc() {
  recover_joint_lift_torque_ = 0.0f;
  const bool all_enabled = lf_joint_.IsDriverReady() &&
                           lb_joint_.IsDriverReady() &&
                           rf_joint_.IsDriverReady() &&
                           rb_joint_.IsDriverReady();

  if (recover_timer < k_recover_timeout)
    recover_timer++;
  if (recover_sub_timer_ < k_recover_timeout)
    recover_sub_timer_++;

  // A recovery attempt must not apply torque indefinitely.
  if (recover_timer >= k_recover_timeout) {
    recover_failed_ = true;
    estop_reason_ = 3;
    ChangeState(STATE_ESTOP);
    StopMotor();
    return;
  }
  if (recover_enable_pending_) {
    if (!all_enabled) {
      if (++dm_enable_wait_count_ >= DM_MANUAL_ENABLE_WAIT_TICKS) {
        // The single Enable attempt did not restore all four drivers.  Keep
        // the original diagnostic masks/reason and require another explicit
        // physical ESTOP -> RECOVER edge for a new attempt.
        ChangeState(STATE_ESTOP);
      }
      StopMotor();
      return;
    }
    recover_enable_pending_ = false;
    dm_enable_wait_count_ = 0U;
  }

  // Body-axis vertical projections remain well-defined around Euler wrapping.
  float forward_projection = INS.xn[2];
  float lateral_projection = INS.yn[2];
  float up_projection = INS.zn[2];
  if (fabsf(forward_projection) + fabsf(lateral_projection) +
          fabsf(up_projection) <
      0.5f) {
    forward_projection = -sinf(INS.Pitch);
    lateral_projection = sinf(INS.Roll) * cosf(INS.Pitch);
    up_projection = cosf(INS.Roll) * cosf(INS.Pitch);
  }
  const bool upright = (up_projection > k_recover_upright_cos) &&
                       (fabsf(INS.Roll) < roll_max) &&
                       (fabsf(INS.Pitch) < pitch_max);
  l_wheel_T_ = 0.0f;
  r_wheel_T_ = 0.0f;
  dist_ = 0.0f;
  target_dist_ = 0.0f;

  switch (recover_sub_status_) {
  case RECOVER_SETTLE:
    left_leg_F_ = right_leg_F_ = 0.0f;
    left_leg_T_ = right_leg_T_ = 0.0f;
    TorCalc();
    if (recover_sub_timer_ >= k_recover_settle_ticks) {
      if (upright) {
        recover_sub_status_ = RECOVER_CENTER_LEGS;
      } else {
        // Any invalid pitch/roll posture first enters a symmetric leg-sync
        // stage. Sweeping two legs from unrelated initial angles can twist the
        // chassis and overload one linkage when it contacts the terrain first.
        recover_sub_status_ = RECOVER_ROLL_TO_SAGITTAL;
      }
      recover_sub_timer_ = 0;
      recover_sub_stable_count_ = 0;
    }
    break;

  case RECOVER_ROLL_TO_SAGITTAL: {
    // This substate is now the safe synchronization stage. Unwrap the right
    // leg near the left leg and freeze their circular mean as a common target.
    if (recover_sub_timer_ == 1) {
      float right_near_left = right_leg_.GetPhi0();
      while (right_near_left - left_leg_.GetPhi0() > PI)
        right_near_left -= 2.0f * PI;
      while (right_near_left - left_leg_.GetPhi0() < -PI)
        right_near_left += 2.0f * PI;
      recover_sweep_phi_ref_ = 0.5f * (left_leg_.GetPhi0() + right_near_left);
    }
    float sync_measure_l = left_leg_.GetPhi0();
    float sync_measure_r = right_leg_.GetPhi0();
    while (sync_measure_l - recover_sweep_phi_ref_ > PI)
      sync_measure_l -= 2.0f * PI;
    while (sync_measure_l - recover_sweep_phi_ref_ < -PI)
      sync_measure_l += 2.0f * PI;
    while (sync_measure_r - recover_sweep_phi_ref_ > PI)
      sync_measure_r -= 2.0f * PI;
    while (sync_measure_r - recover_sweep_phi_ref_ < -PI)
      sync_measure_r += 2.0f * PI;
    const float sync_torque_target_l = ClampAbs(
        k_recover_sync_phi_kp * (recover_sweep_phi_ref_ - sync_measure_l) -
            k_recover_sync_phi_kd * left_leg_.GetPhi0Speed(),
        k_recover_sync_torque_max);
    const float sync_torque_target_r = ClampAbs(
        k_recover_sync_phi_kp * (recover_sweep_phi_ref_ - sync_measure_r) -
            k_recover_sync_phi_kd * right_leg_.GetPhi0Speed(),
        k_recover_sync_torque_max);
    left_leg_T_ = SlewTowards(left_leg_T_, sync_torque_target_l,
                              k_recover_sweep_torque_slew_per_tick);
    right_leg_T_ = SlewTowards(right_leg_T_, sync_torque_target_r,
                               k_recover_sweep_torque_slew_per_tick);
    const float sync_len_err_l = k_recover_sync_leg_len - left_leg_.GetLegLen();
    const float sync_len_err_r =
        k_recover_sync_leg_len - right_leg_.GetLegLen();
    const float sync_len_kp_l = sync_len_err_l < 0.0f
                                    ? k_recover_pose_retract_len_kp
                                    : k_recover_pose_len_kp;
    const float sync_len_kp_r = sync_len_err_r < 0.0f
                                    ? k_recover_pose_retract_len_kp
                                    : k_recover_pose_len_kp;
    left_leg_F_ =
        ClampRange(sync_len_kp_l * sync_len_err_l -
                       k_recover_pose_len_kd * left_leg_.GetLegSpeed(),
                   k_recover_pose_force_min, k_recover_pose_force_max);
    right_leg_F_ =
        ClampRange(sync_len_kp_r * sync_len_err_r -
                       k_recover_pose_len_kd * right_leg_.GetLegSpeed(),
                   k_recover_pose_force_min, k_recover_pose_force_max);
    recover_dynamic_len_ref_ = k_recover_sync_leg_len;
    TorCalc();

    const bool legs_synchronized =
        (fabsf(sync_measure_l - sync_measure_r) < k_recover_sync_pair_err) &&
        (fabsf(left_leg_.GetPhi0Speed()) < k_recover_sync_speed_ok) &&
        (fabsf(right_leg_.GetPhi0Speed()) < k_recover_sync_speed_ok);
    if (legs_synchronized) {
      if (recover_sub_stable_count_ < k_recover_side_exit_hold)
        recover_sub_stable_count_++;
    } else {
      recover_sub_stable_count_ = 0;
    }
    if (recover_sub_stable_count_ >= k_recover_sync_hold) {
      left_leg_len_.Clear();
      right_leg_len_.Clear();
      left_leg_phi0.Clear();
      right_leg_phi0.Clear();
      recover_sweep_phi_ref_ = 0.5f * (sync_measure_l + sync_measure_r);
      recover_sweep_phi_goal_ =
          up_projection < -0.20f ? 1.5f * PI : k_recover_phi0_target;
      while (recover_sweep_phi_goal_ <= recover_sweep_phi_ref_)
        recover_sweep_phi_goal_ += 2.0f * PI;
      recover_sub_status_ = RECOVER_SWEEP;
      recover_sub_timer_ = 0;
      recover_sub_stable_count_ = 0;
    }
    break;
  }

  case RECOVER_SWEEP: {
    const bool pose_capture_candidate =
        upright &&
        (fabsf(lateral_projection) < k_recover_side_exit_projection) &&
        (fabsf(INS.Gyro[0]) < k_recover_pose_capture_rate) &&
        (fabsf(INS.Gyro[1]) < k_recover_pose_capture_rate);
    float sweep_measure_l = left_leg_.GetPhi0();
    float sweep_measure_r = right_leg_.GetPhi0();
    while (sweep_measure_l - recover_sweep_phi_ref_ > PI)
      sweep_measure_l -= 2.0f * PI;
    while (sweep_measure_l - recover_sweep_phi_ref_ < -PI)
      sweep_measure_l += 2.0f * PI;
    while (sweep_measure_r - recover_sweep_phi_ref_ > PI)
      sweep_measure_r -= 2.0f * PI;
    while (sweep_measure_r - recover_sweep_phi_ref_ < -PI)
      sweep_measure_r += 2.0f * PI;

    // Advance one shared reference at a fixed low speed. If either leg is
    // delayed by terrain contact, pause the reference until both catch up.
    const float max_tracking_error =
        fmaxf(fabsf(recover_sweep_phi_ref_ - sweep_measure_l),
              fabsf(recover_sweep_phi_ref_ - sweep_measure_r));
    float sweep_speed_scale =
        (k_recover_sweep_tracking_pause - max_tracking_error) /
        (k_recover_sweep_tracking_pause - k_recover_sweep_tracking_slow);
    if (sweep_speed_scale < 0.0f)
      sweep_speed_scale = 0.0f;
    if (sweep_speed_scale > 1.0f)
      sweep_speed_scale = 1.0f;
    if (!pose_capture_candidate && sweep_speed_scale > 0.0f) {
      recover_sweep_phi_ref_ +=
          k_recover_sweep_speed_per_tick * sweep_speed_scale;
      if (recover_sweep_phi_ref_ > recover_sweep_phi_goal_)
        recover_sweep_phi_ref_ = recover_sweep_phi_goal_;
    }
    if (recover_sweep_phi_ref_ >= recover_sweep_phi_goal_ &&
        !pose_capture_candidate)
      recover_sweep_phi_goal_ += 2.0f * PI;

    const float pair_error = sweep_measure_r - sweep_measure_l;
    const float pair_speed_error =
        right_leg_.GetPhi0Speed() - left_leg_.GetPhi0Speed();
    const float sync_correction = k_recover_sweep_sync_kp * pair_error +
                                  k_recover_sweep_sync_kd * pair_speed_error;
    const float sweep_torque_target_l = ClampAbs(
        k_recover_sweep_phi_kp * (recover_sweep_phi_ref_ - sweep_measure_l) -
            k_recover_sweep_phi_kd * left_leg_.GetPhi0Speed() + sync_correction,
        k_recover_sweep_torque_max);
    const float sweep_torque_target_r = ClampAbs(
        k_recover_sweep_phi_kp * (recover_sweep_phi_ref_ - sweep_measure_r) -
            k_recover_sweep_phi_kd * right_leg_.GetPhi0Speed() -
            sync_correction,
        k_recover_sweep_torque_max);
    left_leg_T_ = SlewTowards(left_leg_T_, sweep_torque_target_l,
                              k_recover_sweep_torque_slew_per_tick);
    right_leg_T_ = SlewTowards(right_leg_T_, sweep_torque_target_r,
                               k_recover_sweep_torque_slew_per_tick);
    const float sweep_len_err_l =
        k_recover_sweep_leg_len - left_leg_.GetLegLen();
    const float sweep_len_err_r =
        k_recover_sweep_leg_len - right_leg_.GetLegLen();
    const float sweep_len_kp_l = sweep_len_err_l < 0.0f
                                     ? k_recover_pose_retract_len_kp
                                     : k_recover_pose_len_kp;
    const float sweep_len_kp_r = sweep_len_err_r < 0.0f
                                     ? k_recover_pose_retract_len_kp
                                     : k_recover_pose_len_kp;
    left_leg_F_ =
        ClampRange(sweep_len_kp_l * sweep_len_err_l -
                       k_recover_pose_len_kd * left_leg_.GetLegSpeed(),
                   k_recover_pose_force_min, k_recover_pose_force_max);
    right_leg_F_ =
        ClampRange(sweep_len_kp_r * sweep_len_err_r -
                       k_recover_pose_len_kd * right_leg_.GetLegSpeed(),
                   k_recover_pose_force_min, k_recover_pose_force_max);
    recover_dynamic_len_ref_ = k_recover_sweep_leg_len;
    TorCalc();

    if (pose_capture_candidate) {
      if (recover_sub_stable_count_ < k_recover_pose_capture_hold)
        recover_sub_stable_count_++;
    } else {
      recover_sub_stable_count_ = 0;
    }
    if (recover_sub_stable_count_ >= k_recover_pose_capture_hold) {
      left_leg_phi0.Clear();
      right_leg_phi0.Clear();
      // Re-enter the same settle -> centre -> balance path used when recovery
      // starts from an already-correct Pitch/Roll pose. This deliberately
      // discards sweep momentum instead of handing its transient state
      // directly to the stand controller.
      recover_sub_status_ = RECOVER_SETTLE;
      recover_sub_timer_ = 0;
      recover_sub_stable_count_ = 0;
      recover_pose_lost_count_ = 0;
      recover_center_wheel_count_ = 0;
      recover_capture_pulse_count_ = 0;
      recover_wheel_torque_lpf_l_ = 0.0f;
      recover_wheel_torque_lpf_r_ = 0.0f;
      recover_timer = 0;
    }
    break;
  }

  case RECOVER_CENTER_LEGS: {
    // Once the body is upright the tail/guard and the two wheels can still
    // form a static three-point support.  Leg torque alone cannot move the
    // centre of mass across the wheel axle, so use the proven normal-mode LQR
    // wheel direction here.  Ramp and clamp it because recovery can enter with
    // much larger state errors than normal balance mode.
    const bool center_pose_lost =
        (up_projection < k_recover_center_fallback_projection) ||
        (fabsf(INS.Roll) > k_recover_center_fallback_angle) ||
        (fabsf(INS.Pitch) > k_recover_center_fallback_angle);
    if (center_pose_lost) {
      if (recover_pose_lost_count_ < k_recover_center_fallback_hold)
        recover_pose_lost_count_++;
    } else {
      recover_pose_lost_count_ = 0;
    }
    if (recover_pose_lost_count_ >= k_recover_center_fallback_hold) {
      // Capture was lost before balance hand-off. Stop all capture outputs and
      // return to symmetric alignment instead of letting the short-leg and
      // wheel controllers fight a body that has fallen out of their domain.
      l_wheel_T_ = r_wheel_T_ = 0.0f;
      left_leg_F_ = right_leg_F_ = 0.0f;
      left_leg_T_ = right_leg_T_ = 0.0f;
      recover_center_wheel_count_ = 0;
      recover_capture_pulse_count_ = 0;
      recover_capture_pitch_sign_ = 0.0f;
      recover_wheel_torque_lpf_l_ = 0.0f;
      recover_wheel_torque_lpf_r_ = 0.0f;
      recover_pose_lost_count_ = 0;
      recover_sub_status_ = RECOVER_ROLL_TO_SAGITTAL;
      recover_sub_timer_ = 0;
      recover_sub_stable_count_ = 0;
      TorCalc();
      break;
    }

    float recover_pitch_comp = k_recover_center_pitch_comp_gain * INS.Pitch;
    if (recover_pitch_comp > k_recover_center_pitch_comp_max)
      recover_pitch_comp = k_recover_center_pitch_comp_max;
    if (recover_pitch_comp < -k_recover_center_pitch_comp_max)
      recover_pitch_comp = -k_recover_center_pitch_comp_max;
    // Keep the virtual legs close to the gravity direction instead of merely
    // perpendicular to a body that is still pitched during recovery.
    const float recover_center_phi_target = 0.5f * PI - recover_pitch_comp;

    if (recover_sub_timer_ == 1) {
      // Start from the measured length to avoid a force step when sweep hands
      // over to centering, then move toward the short support pose gradually.
      recover_dynamic_len_ref_ =
          fminf(0.5f * (left_leg_.GetLegLen() + right_leg_.GetLegLen()),
                k_recover_center_entry_len_max);
    }
    const bool center_lift_active =
        recover_center_wheel_count_ >= k_recover_wheel_ready_hold;
    const float center_len_target = center_lift_active
                                        ? k_recover_capture_stand_len
                                        : k_recover_capture_prep_len;
    const float center_len_slew = center_lift_active
                                      ? k_recover_capture_stand_slew_per_tick
                                      : k_recover_center_len_slew_per_tick;
    if (recover_dynamic_len_ref_ < center_len_target) {
      recover_dynamic_len_ref_ += center_len_slew;
      if (recover_dynamic_len_ref_ > center_len_target)
        recover_dynamic_len_ref_ = center_len_target;
    } else if (recover_dynamic_len_ref_ > center_len_target) {
      recover_dynamic_len_ref_ -= center_len_slew;
      if (recover_dynamic_len_ref_ < center_len_target)
        recover_dynamic_len_ref_ = center_len_target;
    }

    const bool wheel_geometry_ready =
        (up_projection > k_recover_center_gate_up_projection) &&
        (left_leg_.GetLegLen() > k_recover_wheel_ready_len_min) &&
        (right_leg_.GetLegLen() > k_recover_wheel_ready_len_min) &&
        (fabsf(left_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_wheel_ready_phi_err) &&
        (fabsf(right_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_wheel_ready_phi_err) &&
        (fabsf(left_leg_.GetLegSpeed()) < k_recover_wheel_ready_len_speed) &&
        (fabsf(right_leg_.GetLegSpeed()) < k_recover_wheel_ready_len_speed) &&
        (fabsf(left_leg_.GetPhi0Speed()) < k_recover_wheel_ready_phi_speed) &&
        (fabsf(right_leg_.GetPhi0Speed()) < k_recover_wheel_ready_phi_speed);
    const bool wheel_gate_latched =
        recover_center_wheel_count_ >= k_recover_wheel_ready_hold;
    const bool wheel_capture_safe =
        (up_projection > k_recover_wheel_latched_up_projection) &&
        (left_leg_.GetLegLen() > k_recover_wheel_latched_len_min) &&
        (left_leg_.GetLegLen() < k_recover_wheel_latched_len_max) &&
        (right_leg_.GetLegLen() > k_recover_wheel_latched_len_min) &&
        (right_leg_.GetLegLen() < k_recover_wheel_latched_len_max) &&
        (fabsf(left_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_wheel_latched_phi_err) &&
        (fabsf(right_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_wheel_latched_phi_err);
    if ((wheel_gate_latched && wheel_capture_safe) ||
        (!wheel_gate_latched && wheel_geometry_ready)) {
      if (recover_center_wheel_count_ <
          k_recover_wheel_ready_hold + k_recover_center_wheel_ramp_ticks) {
        recover_center_wheel_count_++;
      }
    } else {
      recover_center_wheel_count_ = 0;
    }
    const bool wheel_gate_active =
        recover_center_wheel_count_ >= k_recover_wheel_ready_hold;
    if (!wheel_gate_active) {
      recover_capture_pulse_count_ = 0;
      recover_capture_pitch_sign_ = 0.0f;
    }
    const bool capture_leg_ready =
        wheel_gate_active &&
        (fabsf(left_leg_.GetLegLen() - k_recover_capture_stand_len) <
         k_recover_capture_leg_ready_err) &&
        (fabsf(right_leg_.GetLegLen() - k_recover_capture_stand_len) <
         k_recover_capture_leg_ready_err) &&
        (fabsf(left_leg_.GetLegSpeed()) < k_recover_capture_leg_ready_speed) &&
        (fabsf(right_leg_.GetLegSpeed()) < k_recover_capture_leg_ready_speed);
    if (recover_capture_pulse_count_ == 0) {
      if (capture_leg_ready) {
        recover_capture_pulse_count_ = 1;
        recover_capture_pitch_sign_ = INS.Pitch >= 0.0f ? 1.0f : -1.0f;
      }
    } else if (recover_capture_pulse_count_ <
               1 + k_recover_wheel_breakaway_delay_ticks +
                   k_recover_wheel_breakaway_duration_ticks) {
      recover_capture_pulse_count_++;
    }
    const uint32_t capture_pulse_ticks =
        recover_capture_pulse_count_ > 0 ? recover_capture_pulse_count_ - 1 : 0;
    const bool breakaway_window =
        (recover_capture_pulse_count_ > 0) &&
        (recover_capture_pitch_sign_ * INS.Pitch > 0.0f) &&
        (capture_pulse_ticks >= k_recover_wheel_breakaway_delay_ticks) &&
        (capture_pulse_ticks < k_recover_wheel_breakaway_delay_ticks +
                                   k_recover_wheel_breakaway_duration_ticks);

    if (recover_center_wheel_count_ >= k_recover_wheel_ready_hold) {
      LQRCalc();
      float wheel_blend =
          (recover_center_wheel_count_ - k_recover_wheel_ready_hold) /
          k_recover_center_wheel_ramp_ticks;
      if (wheel_blend > 1.0f)
        wheel_blend = 1.0f;
      float wheel_target_l =
          wheel_blend *
          ClampAbs(lqr_body_.GetWheelTorL(), k_recover_center_wheel_torque_max);
      float wheel_target_r =
          wheel_blend *
          ClampAbs(lqr_body_.GetWheelTorR(), k_recover_center_wheel_torque_max);
      const float capture_common = 0.5f * (wheel_target_l + wheel_target_r);

      // Blend continuously from posture capture to velocity braking. A hard
      // switch at the speed limit creates a limit cycle: LQR accelerates until
      // the threshold, braking reverses it, then LQR immediately takes over.
      float brake_blend =
          (fabsf(vel_) - k_recover_wheel_speed_soft_start) /
          (k_recover_wheel_speed_limit - k_recover_wheel_speed_soft_start);
      if (brake_blend < 0.0f)
        brake_blend = 0.0f;
      if (brake_blend > 1.0f)
        brake_blend = 1.0f;
      const float brake_torque = ClampAbs(-k_recover_wheel_speed_damping * vel_,
                                          k_recover_center_wheel_torque_max);
      wheel_target_l =
          (1.0f - brake_blend) * wheel_target_l + brake_blend * brake_torque;
      wheel_target_r =
          (1.0f - brake_blend) * wheel_target_r + brake_blend * brake_torque;

      // Static friction and the auxiliary roller can absorb a very small
      // smooth LQR command indefinitely. Preserve a modest common breakaway
      // torque in the requested capture direction, then fade it to zero before
      // the speed limit so continuous braking retains full authority.
      float breakaway_scale =
          (k_recover_wheel_speed_limit - fabsf(vel_)) /
          (k_recover_wheel_speed_limit - k_recover_wheel_breakaway_fade_start);
      if (fabsf(vel_) <= k_recover_wheel_breakaway_fade_start)
        breakaway_scale = 1.0f;
      if (breakaway_scale < 0.0f)
        breakaway_scale = 0.0f;
      if (breakaway_scale > 1.0f)
        breakaway_scale = 1.0f;
      float pitch_scale =
          (fabsf(INS.Pitch) - k_recover_wheel_breakaway_pitch_off) /
          (k_recover_wheel_breakaway_pitch_full -
           k_recover_wheel_breakaway_pitch_off);
      if (pitch_scale < 0.0f)
        pitch_scale = 0.0f;
      if (pitch_scale > 1.0f)
        pitch_scale = 1.0f;
      breakaway_scale *= pitch_scale;
      if (breakaway_window && fabsf(capture_common) > 0.05f &&
          breakaway_scale > 0.0f) {
        const float min_common = (capture_common > 0.0f ? 1.0f : -1.0f) *
                                 k_recover_wheel_breakaway_torque *
                                 breakaway_scale;
        const float current_common = 0.5f * (wheel_target_l + wheel_target_r);
        if ((min_common > 0.0f && current_common < min_common) ||
            (min_common < 0.0f && current_common > min_common)) {
          const float common_correction = min_common - current_common;
          wheel_target_l = ClampAbs(wheel_target_l + common_correction,
                                    k_recover_center_wheel_torque_max);
          wheel_target_r = ClampAbs(wheel_target_r + common_correction,
                                    k_recover_center_wheel_torque_max);
        }
      }
      // Recovery geometry and contact changes make the raw LQR command very
      // sensitive to derivative noise.  Filter it here so the wheel receives
      // a useful mean capture torque instead of alternating saturation.
      const float lpf_alpha_l =
          (recover_wheel_torque_lpf_l_ * wheel_target_l < 0.0f &&
           fabsf(vel_) > 0.05f)
              ? k_recover_wheel_torque_reverse_alpha
              : k_recover_wheel_torque_lpf_alpha;
      const float lpf_alpha_r =
          (recover_wheel_torque_lpf_r_ * wheel_target_r < 0.0f &&
           fabsf(vel_) > 0.05f)
              ? k_recover_wheel_torque_reverse_alpha
              : k_recover_wheel_torque_lpf_alpha;
      recover_wheel_torque_lpf_l_ +=
          lpf_alpha_l * (wheel_target_l - recover_wheel_torque_lpf_l_);
      recover_wheel_torque_lpf_r_ +=
          lpf_alpha_r * (wheel_target_r - recover_wheel_torque_lpf_r_);
      l_wheel_T_ = recover_wheel_torque_lpf_l_;
      r_wheel_T_ = recover_wheel_torque_lpf_r_;

      // If capture starts running away, override the posture command with a
      // stronger bounded common-mode brake. The ordinary 1.5 N.m capture limit
      // was unable to arrest the measured rearward speed before rear contact.
      if (fabsf(vel_) > k_recover_emergency_speed) {
        const float emergency_brake =
            ClampAbs(-k_recover_emergency_brake_gain * vel_,
                     k_recover_emergency_brake_torque);
        l_wheel_T_ = emergency_brake;
        r_wheel_T_ = emergency_brake;
        recover_wheel_torque_lpf_l_ = emergency_brake;
        recover_wheel_torque_lpf_r_ = emergency_brake;
      }
    } else {
      recover_wheel_torque_lpf_l_ = 0.0f;
      recover_wheel_torque_lpf_r_ = 0.0f;
    }

    // Use this robot's VMC coordinates for the leg hand-off. The reference
    // robot's joint-angle LQR depends on different zero points and directions.
    const float base_ff_left =
        k_gravity_comp * arm_cos_f32(left_leg_.GetTheta());
    const float base_ff_right =
        k_gravity_comp * arm_cos_f32(right_leg_.GetTheta());
    // Leg preparation must be able to lift the mechanism into the wheel-gate
    // range. Requiring the gate first creates a circular dependency after a
    // terrain-assisted flip, where the loaded legs settle below 0.15 m.
    const bool direct_stand_active = wheel_gate_active;
    const bool leg_prepare_active =
        !direct_stand_active && (recover_capture_pulse_count_ == 0);
    float direct_stand_boost_scale =
        (k_recover_capture_stand_len -
         0.5f * (left_leg_.GetLegLen() + right_leg_.GetLegLen())) /
        (k_recover_capture_stand_len - k_recover_capture_prep_len);
    if (!direct_stand_active)
      direct_stand_boost_scale = 0.0f;
    if (direct_stand_boost_scale < 0.0f)
      direct_stand_boost_scale = 0.0f;
    if (direct_stand_boost_scale > 1.0f)
      direct_stand_boost_scale = 1.0f;
    float left_leg_boost_scale =
        (k_recover_capture_prep_len - left_leg_.GetLegLen()) /
        k_recover_capture_leg_boost_err_range;
    float right_leg_boost_scale =
        (k_recover_capture_prep_len - right_leg_.GetLegLen()) /
        k_recover_capture_leg_boost_err_range;
    if (!leg_prepare_active) {
      left_leg_boost_scale = 0.0f;
      right_leg_boost_scale = 0.0f;
    }
    if (left_leg_boost_scale < 0.0f)
      left_leg_boost_scale = 0.0f;
    if (left_leg_boost_scale > 1.0f)
      left_leg_boost_scale = 1.0f;
    if (right_leg_boost_scale < 0.0f)
      right_leg_boost_scale = 0.0f;
    if (right_leg_boost_scale > 1.0f)
      right_leg_boost_scale = 1.0f;
    const float left_capture_kd =
        direct_stand_active
            ? k_recover_capture_stand_kd
            : k_recover_center_len_kd +
                  left_leg_boost_scale *
                      (k_recover_capture_leg_len_kd - k_recover_center_len_kd);
    const float right_capture_kd =
        direct_stand_active
            ? k_recover_capture_stand_kd
            : k_recover_center_len_kd +
                  right_leg_boost_scale *
                      (k_recover_capture_leg_len_kd - k_recover_center_len_kd);
    const float left_capture_force_max =
        direct_stand_active
            ? k_recover_capture_stand_force_max
            : k_recover_center_force_max +
                  left_leg_boost_scale * (k_recover_capture_leg_force_max -
                                          k_recover_center_force_max);
    const float right_capture_force_max =
        direct_stand_active
            ? k_recover_capture_stand_force_max
            : k_recover_center_force_max +
                  right_leg_boost_scale * (k_recover_capture_leg_force_max -
                                           k_recover_center_force_max);
    // Extension remains compliant, but an over-long leg needs active negative
    // axial force. Otherwise gravity feed-forward cancels the retract command
    // and leaves the robot propped on an auxiliary roller indefinitely.
    const float left_len_err = recover_dynamic_len_ref_ - left_leg_.GetLegLen();
    const float right_len_err =
        recover_dynamic_len_ref_ - right_leg_.GetLegLen();
    const bool left_retracting =
        left_len_err < -k_recover_center_retract_threshold;
    const bool right_retracting =
        right_len_err < -k_recover_center_retract_threshold;
    // If a loaded leg passes the vertical target, the contact force can keep
    // opening the linkage even though the ordinary PD is asking it to return.
    // Apply the stronger branch only on the overrun side of the target; this
    // brakes the leg back toward the capturable sector without making the
    // normal approach unnecessarily violent.
    const bool left_phi_overrun =
        (left_leg_.GetPhi0() - recover_center_phi_target) >
        k_recover_center_overrun_angle;
    const bool right_phi_overrun =
        (right_leg_.GetPhi0() - recover_center_phi_target) >
        k_recover_center_overrun_angle;
    const float left_len_kp =
        left_retracting
            ? (left_phi_overrun ? k_recover_center_overrun_retract_kp
                                : k_recover_center_retract_len_kp)
            : k_recover_center_len_kp;
    const float right_len_kp =
        right_retracting
            ? (right_phi_overrun ? k_recover_center_overrun_retract_kp
                                 : k_recover_center_retract_len_kp)
            : k_recover_center_len_kp;
    const float left_ff =
        left_retracting
            ? (left_phi_overrun
                   ? 0.0f
                   : k_recover_center_retract_ff_scale * base_ff_left)
            : base_ff_left;
    const float right_ff =
        right_retracting
            ? (right_phi_overrun
                   ? 0.0f
                   : k_recover_center_retract_ff_scale * base_ff_right)
            : base_ff_right;
    left_leg_F_ = ClampRange(
        left_len_kp * left_len_err - left_capture_kd * left_leg_.GetLegSpeed() +
            left_ff + left_leg_boost_scale * k_recover_capture_leg_boost_force +
            direct_stand_boost_scale * k_recover_capture_stand_boost_force,
        left_phi_overrun ? k_recover_center_overrun_force_min
                         : k_recover_center_retract_force_min,
        left_capture_force_max);
    right_leg_F_ = ClampRange(
        right_len_kp * right_len_err -
            right_capture_kd * right_leg_.GetLegSpeed() + right_ff +
            right_leg_boost_scale * k_recover_capture_leg_boost_force +
            direct_stand_boost_scale * k_recover_capture_stand_boost_force,
        right_phi_overrun ? k_recover_center_overrun_force_min
                          : k_recover_center_retract_force_min,
        right_capture_force_max);

    // Recovery previously had no roll loop. Add a bounded differential support
    // force so a small left/right disturbance does not grow while the wheels
    // are capturing pitch. Keep this below the normal-mode roll authority.
    roll_comp_.SetRef(0.0f);
    roll_comp_.SetMeasure(INS.Roll);
    const float recover_roll_raw =
        ClampAbs(-kRecoverRollKp * INS.Roll - kRecoverRollKd * INS.Gyro[0],
                 k_recover_roll_force_max);
    roll_force_cmd_ +=
        k_recover_roll_force_lpf_alpha * (recover_roll_raw - roll_force_cmd_);
    left_leg_F_ =
        ClampRange(left_leg_F_ + roll_force_cmd_,
                   k_recover_center_retract_force_min, left_capture_force_max);
    right_leg_F_ =
        ClampRange(right_leg_F_ - roll_force_cmd_,
                   k_recover_center_retract_force_min, right_capture_force_max);
    left_leg_T_ =
        ClampAbs((left_phi_overrun ? k_recover_center_overrun_phi_kp
                                   : k_recover_center_phi_kp) *
                         (recover_center_phi_target - left_leg_.GetPhi0()) -
                     (left_phi_overrun ? k_recover_center_overrun_phi_kd
                                       : k_recover_center_phi_kd) *
                         left_leg_.GetPhi0Speed(),
                 left_phi_overrun ? k_recover_center_overrun_torque_max
                                  : k_recover_center_torque_max);
    right_leg_T_ =
        ClampAbs((right_phi_overrun ? k_recover_center_overrun_phi_kp
                                    : k_recover_center_phi_kp) *
                         (recover_center_phi_target - right_leg_.GetPhi0()) -
                     (right_phi_overrun ? k_recover_center_overrun_phi_kd
                                        : k_recover_center_phi_kd) *
                         right_leg_.GetPhi0Speed(),
                 right_phi_overrun ? k_recover_center_overrun_torque_max
                                   : k_recover_center_torque_max);
    TorCalc();

    // At a deeply folded length the VMC axial-force Jacobian has very little
    // extension leverage.  This was already handled in BALANCE, but a new
    // recovery attempt re-enters CENTER and loses that assistance, leaving
    // 70-80 N of virtual axial force without measurable leg extension.  Use
    // the same IK joint-space lift here and blend back to VMC by 0.21 m.
    const float center_joint_lift_span =
        k_recover_joint_lift_release_len - k_recover_joint_lift_full_len;
    float center_joint_lift_blend_l =
        (k_recover_joint_lift_release_len - left_leg_.GetLegLen()) /
        center_joint_lift_span;
    float center_joint_lift_blend_r =
        (k_recover_joint_lift_release_len - right_leg_.GetLegLen()) /
        center_joint_lift_span;
    if (center_joint_lift_blend_l < 0.0f)
      center_joint_lift_blend_l = 0.0f;
    if (center_joint_lift_blend_l > 1.0f)
      center_joint_lift_blend_l = 1.0f;
    if (center_joint_lift_blend_r < 0.0f)
      center_joint_lift_blend_r = 0.0f;
    if (center_joint_lift_blend_r > 1.0f)
      center_joint_lift_blend_r = 1.0f;
    if (center_joint_lift_blend_l > 0.0f || center_joint_lift_blend_r > 0.0f) {
      float direct_l1 = 0.0f;
      float direct_l2 = 0.0f;
      float direct_r1 = 0.0f;
      float direct_r2 = 0.0f;
      RecoverLegJointLQR(left_leg_, recover_center_phi_target,
                         recover_dynamic_len_ref_, direct_l1, direct_l2);
      RecoverLegJointLQR(right_leg_, recover_center_phi_target,
                         recover_dynamic_len_ref_, direct_r1, direct_r2);
      direct_l1 = ClampAbs(direct_l1, k_recover_joint_lift_torque_max);
      direct_l2 = ClampAbs(direct_l2, k_recover_joint_lift_torque_max);
      direct_r1 = ClampAbs(direct_r1, k_recover_joint_lift_torque_max);
      direct_r2 = ClampAbs(direct_r2, k_recover_joint_lift_torque_max);
      recover_joint_lift_torque_ =
          0.25f *
          (center_joint_lift_blend_l * (fabsf(direct_l1) + fabsf(direct_l2)) +
           center_joint_lift_blend_r * (fabsf(direct_r1) + fabsf(direct_r2)));
      left_leg_.SetDirectJointTor(
          (1.0f - center_joint_lift_blend_l) * left_leg_.GetT1() +
              center_joint_lift_blend_l * direct_l1,
          (1.0f - center_joint_lift_blend_l) * left_leg_.GetT2() +
              center_joint_lift_blend_l * direct_l2);
      right_leg_.SetDirectJointTor(
          (1.0f - center_joint_lift_blend_r) * right_leg_.GetT1() +
              center_joint_lift_blend_r * direct_r1,
          (1.0f - center_joint_lift_blend_r) * right_leg_.GetT2() +
              center_joint_lift_blend_r * direct_r2);
      left_leg_F_ *= 1.0f - center_joint_lift_blend_l;
      right_leg_F_ *= 1.0f - center_joint_lift_blend_r;
    }

    const bool geometry_ok =
        (fabsf(left_leg_.GetLegLen() - recover_center_target_len) <
         k_shoutui_length_th) &&
        (fabsf(right_leg_.GetLegLen() - recover_center_target_len) <
         k_shoutui_length_th) &&
        (fabsf(left_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_align_angle_th) &&
        (fabsf(right_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_align_angle_th) &&
        (fabsf(left_leg_.GetLegSpeed()) < k_shoutui_speed_th) &&
        (fabsf(right_leg_.GetLegSpeed()) < k_shoutui_speed_th) &&
        (fabsf(left_leg_.GetPhi0Speed()) < k_phi0speed_ok) &&
        (fabsf(right_leg_.GetPhi0Speed()) < k_phi0speed_ok);
    // A loaded auxiliary roller can prevent the legs from reaching the final
    // length while the body is already passing through the capturable region.
    // Hand full LQR control over at that moment instead of waiting for static
    // geometry that can only be reached after the roller is unloaded.
    const bool capture_handoff_ok =
        GetRecoverWheelReady() && upright &&
        (left_leg_.GetLegLen() > k_recover_capture_handoff_len_min) &&
        (right_leg_.GetLegLen() > k_recover_capture_handoff_len_min) &&
        (fabsf(INS.Pitch) < k_recover_capture_handoff_pitch) &&
        (fabsf(vel_) < k_recover_capture_handoff_speed) &&
        (fabsf(left_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_capture_handoff_phi_err) &&
        (fabsf(right_leg_.GetPhi0() - recover_center_phi_target) <
         k_recover_capture_handoff_phi_err);
    if (upright && (geometry_ok || capture_handoff_ok)) {
      if (recover_sub_stable_count_ < k_recover_center_hold)
        recover_sub_stable_count_++;
    } else {
      recover_sub_stable_count_ = 0;
    }
    const bool center_geometry_complete =
        geometry_ok && (recover_sub_stable_count_ >= k_recover_center_hold);
    const bool dynamic_capture_ready =
        capture_handoff_ok &&
        (recover_sub_stable_count_ >= k_recover_capture_handoff_hold);
    if (center_geometry_complete || dynamic_capture_ready) {
      recover_leg_retracted_ = true;
      recover_yaw_target_ = INS.Yaw;
      recover_sub_status_ = RECOVER_BALANCE;
      recover_sub_timer_ = 0;
      recover_sub_stable_count_ = 0;
      recover_balance_cnt_ = 0;
      recover_yaw_cnt_ = 0;
      // The sweep/flip phase may legitimately consume most of the attempt
      // watchdog. Give the newly captured balance phase its own full window.
      recover_timer = 0;
    }
    break;
  }

  case RECOVER_BALANCE: {
    LQRCalc();
    if (recover_dynamic_len_ref_ < k_recover_balance_capture_len) {
      recover_dynamic_len_ref_ += k_recover_balance_len_slew_per_tick;
      if (recover_dynamic_len_ref_ > k_recover_balance_capture_len)
        recover_dynamic_len_ref_ = k_recover_balance_capture_len;
    } else if (recover_dynamic_len_ref_ > k_recover_balance_capture_len) {
      recover_dynamic_len_ref_ -= k_recover_balance_len_slew_per_tick;
      if (recover_dynamic_len_ref_ < k_recover_balance_capture_len)
        recover_dynamic_len_ref_ = k_recover_balance_capture_len;
    }
    const float base_ff_left =
        k_gravity_comp * arm_cos_f32(left_leg_.GetTheta());
    const float base_ff_right =
        k_gravity_comp * arm_cos_f32(right_leg_.GetTheta());
    SynthesizeMotion();
    // Start compliant at wheel capture, then progressively approach the
    // support authority of normal leg control. A fixed soft gain leaves a
    // 0.14-0.16 m leg unable to unload the auxiliary roller even with a
    // 0.23 m reference.
    float support_blend =
        (recover_sub_timer_ - k_recover_balance_support_ramp_delay) /
        k_recover_balance_support_ramp_ticks;
    if (support_blend < 0.0f)
      support_blend = 0.0f;
    if (support_blend > 1.0f)
      support_blend = 1.0f;
    const float balance_len_kp =
        k_recover_balance_len_kp +
        support_blend *
            (k_recover_balance_len_kp_final - k_recover_balance_len_kp);
    const float balance_force_max =
        k_recover_balance_force_max +
        support_blend *
            (k_recover_balance_force_max_final - k_recover_balance_force_max);
    // Keep recovery leg length compliant and explicitly bounded. The normal
    // leg-length PID can generate a large force pulse when contact changes at
    // the exact moment state 4 takes over.
    left_leg_F_ = ClampRange(
        balance_len_kp * (recover_dynamic_len_ref_ - left_leg_.GetLegLen()) -
            k_recover_balance_len_kd * left_leg_.GetLegSpeed() + base_ff_left,
        k_recover_balance_force_min, balance_force_max);
    right_leg_F_ = ClampRange(
        balance_len_kp * (recover_dynamic_len_ref_ - right_leg_.GetLegLen()) -
            k_recover_balance_len_kd * right_leg_.GetLegSpeed() + base_ff_right,
        k_recover_balance_force_min, balance_force_max);

    roll_comp_.SetRef(0.0f);
    roll_comp_.SetMeasure(INS.Roll);
    const float recover_roll_raw =
        ClampAbs(-kRecoverRollKp * INS.Roll - kRecoverRollKd * INS.Gyro[0],
                 k_recover_roll_force_max);
    roll_force_cmd_ +=
        k_recover_roll_force_lpf_alpha * (recover_roll_raw - roll_force_cmd_);
    left_leg_F_ = ClampRange(left_leg_F_ + roll_force_cmd_,
                             k_recover_balance_force_min, balance_force_max);
    right_leg_F_ = ClampRange(right_leg_F_ - roll_force_cmd_,
                              k_recover_balance_force_min, balance_force_max);

    // Blend from the filtered centre-stage torque into full LQR. Limiting the
    // recovery balance torque avoids a step from 1.5 N.m directly to the
    // normal LQR's 10 N.m limit at the instant the auxiliary roller unloads.
    float balance_wheel_blend =
        recover_sub_timer_ / k_recover_balance_wheel_ramp_ticks;
    if (balance_wheel_blend > 1.0f)
      balance_wheel_blend = 1.0f;
    const float balance_lqr_l =
        ClampAbs(l_wheel_T_, k_recover_balance_wheel_torque_max);
    const float balance_lqr_r =
        ClampAbs(r_wheel_T_, k_recover_balance_wheel_torque_max);
    float balance_target_l =
        (1.0f - balance_wheel_blend) * recover_wheel_torque_lpf_l_ +
        balance_wheel_blend * balance_lqr_l;
    float balance_target_r =
        (1.0f - balance_wheel_blend) * recover_wheel_torque_lpf_r_ +
        balance_wheel_blend * balance_lqr_r;

    float balance_brake_blend =
        (fabsf(vel_) - k_recover_balance_speed_soft_start) /
        (k_recover_balance_speed_limit - k_recover_balance_speed_soft_start);
    if (balance_brake_blend < 0.0f)
      balance_brake_blend = 0.0f;
    if (balance_brake_blend > 1.0f)
      balance_brake_blend = 1.0f;
    const float balance_brake_torque =
        ClampAbs(-k_recover_balance_speed_damping * vel_,
                 k_recover_balance_wheel_torque_max);
    balance_target_l = (1.0f - balance_brake_blend) * balance_target_l +
                       balance_brake_blend * balance_brake_torque;
    balance_target_r = (1.0f - balance_brake_blend) * balance_target_r +
                       balance_brake_blend * balance_brake_torque;

    const float balance_alpha_l =
        (recover_wheel_torque_lpf_l_ * balance_target_l < 0.0f &&
         fabsf(vel_) > k_recover_balance_speed_soft_start)
            ? k_recover_balance_reverse_alpha
            : k_recover_wheel_torque_lpf_alpha;
    const float balance_alpha_r =
        (recover_wheel_torque_lpf_r_ * balance_target_r < 0.0f &&
         fabsf(vel_) > k_recover_balance_speed_soft_start)
            ? k_recover_balance_reverse_alpha
            : k_recover_wheel_torque_lpf_alpha;
    recover_wheel_torque_lpf_l_ +=
        balance_alpha_l * (balance_target_l - recover_wheel_torque_lpf_l_);
    recover_wheel_torque_lpf_r_ +=
        balance_alpha_r * (balance_target_r - recover_wheel_torque_lpf_r_);
    l_wheel_T_ = recover_wheel_torque_lpf_l_;
    r_wheel_T_ = recover_wheel_torque_lpf_r_;

    if (fabsf(vel_) > k_recover_emergency_speed) {
      const float emergency_brake =
          ClampAbs(-k_recover_emergency_brake_gain * vel_,
                   k_recover_emergency_brake_torque);
      l_wheel_T_ = emergency_brake;
      r_wheel_T_ = emergency_brake;
      recover_wheel_torque_lpf_l_ = emergency_brake;
      recover_wheel_torque_lpf_r_ = emergency_brake;
    }

    // Continue the same fast pitch-compensated leg swing used in state 3.
    // Only the wheels are handed to LQR here; otherwise the slow LQR leg
    // channel pulls phi0 away from the recovery target immediately after
    // capture and props the body on an auxiliary roller again.
    float balance_pitch_comp = k_recover_center_pitch_comp_gain * INS.Pitch;
    if (balance_pitch_comp > k_recover_center_pitch_comp_max)
      balance_pitch_comp = k_recover_center_pitch_comp_max;
    if (balance_pitch_comp < -k_recover_center_pitch_comp_max)
      balance_pitch_comp = -k_recover_center_pitch_comp_max;
    const float balance_phi_target = 0.5f * PI - balance_pitch_comp;
    left_leg_T_ = ClampAbs(
        k_recover_center_phi_kp * (balance_phi_target - left_leg_.GetPhi0()) -
            k_recover_center_phi_kd * left_leg_.GetPhi0Speed(),
        k_recover_center_torque_max);
    right_leg_T_ = ClampAbs(
        k_recover_center_phi_kp * (balance_phi_target - right_leg_.GetPhi0()) -
            k_recover_center_phi_kd * right_leg_.GetPhi0Speed(),
        k_recover_center_torque_max);

    // During the short-leg capture, keep updating the reference so a yaw
    // angle error cannot build while the wheels are busy balancing pitch.
    if (recover_sub_timer_ <= k_recover_balance_yaw_delay_ticks) {
      recover_yaw_target_ = INS.Yaw;
    }
    float yaw_err = INS.Yaw - recover_yaw_target_;
    if (yaw_err > PI)
      yaw_err -= 2.0f * PI;
    if (yaw_err < -PI)
      yaw_err += 2.0f * PI;
    const float yaw_w =
        ClampAbs(k_recover_yaw_k1 * yaw_err, k_recover_yaw_w_limit);
    float yaw_blend = (recover_sub_timer_ - k_recover_balance_yaw_delay_ticks) /
                      k_recover_balance_yaw_ramp_ticks;
    if (yaw_blend < 0.0f)
      yaw_blend = 0.0f;
    if (yaw_blend > 1.0f)
      yaw_blend = 1.0f;
    const float yaw_tau =
        yaw_blend * ClampAbs(k_recover_yaw_k2 * (INS.YawSpeed + yaw_w),
                             k_recover_balance_yaw_torque_max);
    l_wheel_T_ = ClampAbs(l_wheel_T_ + 0.5f * balance_wheel_blend * yaw_tau,
                          k_recover_balance_wheel_torque_max);
    r_wheel_T_ = ClampAbs(r_wheel_T_ + 0.5f * balance_wheel_blend * yaw_tau,
                          k_recover_balance_wheel_torque_max);
    TorCalc();

    // At very short length the VMC axial-force Jacobian has poor leverage and
    // can drive the joint motors into their 25 N.m clamp without extending the
    // leg (the audible high-frequency buzz). Reuse the earlier IK + joint-LQR
    // stand controller in that region, then blend back to VMC after escaping
    // the folded geometry.
    const float joint_lift_span =
        k_recover_joint_lift_release_len - k_recover_joint_lift_full_len;
    float joint_lift_blend_l =
        (k_recover_joint_lift_release_len - left_leg_.GetLegLen()) /
        joint_lift_span;
    float joint_lift_blend_r =
        (k_recover_joint_lift_release_len - right_leg_.GetLegLen()) /
        joint_lift_span;
    if (joint_lift_blend_l < 0.0f)
      joint_lift_blend_l = 0.0f;
    if (joint_lift_blend_l > 1.0f)
      joint_lift_blend_l = 1.0f;
    if (joint_lift_blend_r < 0.0f)
      joint_lift_blend_r = 0.0f;
    if (joint_lift_blend_r > 1.0f)
      joint_lift_blend_r = 1.0f;
    if (joint_lift_blend_l > 0.0f || joint_lift_blend_r > 0.0f) {
      float direct_l1 = 0.0f;
      float direct_l2 = 0.0f;
      float direct_r1 = 0.0f;
      float direct_r2 = 0.0f;
      RecoverLegJointLQR(left_leg_, balance_phi_target,
                         recover_dynamic_len_ref_, direct_l1, direct_l2);
      RecoverLegJointLQR(right_leg_, balance_phi_target,
                         recover_dynamic_len_ref_, direct_r1, direct_r2);
      direct_l1 = ClampAbs(direct_l1, k_recover_joint_lift_torque_max);
      direct_l2 = ClampAbs(direct_l2, k_recover_joint_lift_torque_max);
      direct_r1 = ClampAbs(direct_r1, k_recover_joint_lift_torque_max);
      direct_r2 = ClampAbs(direct_r2, k_recover_joint_lift_torque_max);
      recover_joint_lift_torque_ =
          0.25f * (joint_lift_blend_l * (fabsf(direct_l1) + fabsf(direct_l2)) +
                   joint_lift_blend_r * (fabsf(direct_r1) + fabsf(direct_r2)));
      left_leg_.SetDirectJointTor(
          (1.0f - joint_lift_blend_l) * left_leg_.GetT1() +
              joint_lift_blend_l * direct_l1,
          (1.0f - joint_lift_blend_l) * left_leg_.GetT2() +
              joint_lift_blend_l * direct_l2);
      right_leg_.SetDirectJointTor(
          (1.0f - joint_lift_blend_r) * right_leg_.GetT1() +
              joint_lift_blend_r * direct_r1,
          (1.0f - joint_lift_blend_r) * right_leg_.GetT2() +
              joint_lift_blend_r * direct_r2);
      // I8 represents the portion of axial VMC force actually retained after
      // the short-leg joint-control blend.
      left_leg_F_ *= 1.0f - joint_lift_blend_l;
      right_leg_F_ *= 1.0f - joint_lift_blend_r;
    }
    recover_state_ = true;

    // The regular NORMAL controller has been verified on hardware to capture
    // and raise the chassis from 0.18 m.  Recovery therefore only has to reach
    // that safe hand-off length; forcing the folded-leg controller all the way
    // to 0.23 m unnecessarily holds the state machine in RECOVER_BALANCE.
    const bool leg_length_ready =
        (left_leg_.GetLegLen() >= k_recover_normal_handoff_min_len) &&
        (right_leg_.GetLegLen() >= k_recover_normal_handoff_min_len) &&
        (fabsf(left_leg_.GetLegSpeed()) <
         k_recover_balance_complete_len_speed) &&
        (fabsf(right_leg_.GetLegSpeed()) <
         k_recover_balance_complete_len_speed);
    const bool body_stable = upright &&
                             (fabsf(INS.Gyro[0]) < k_recover_body_rate_ok) &&
                             (fabsf(INS.Gyro[1]) < k_recover_body_rate_ok) &&
                             (fabsf(INS.YawSpeed) < k_recover_body_rate_ok) &&
                             (fabsf(vel_) < k_recover_balance_speed_limit) &&
                             leg_length_ready && !GetOffGround();
    if (body_stable) {
      if (recover_balance_cnt_ < k_recover_pitch_balance_hold)
        recover_balance_cnt_++;
    } else {
      recover_balance_cnt_ = 0;
      recover_yaw_cnt_ = 0;
    }
    // Absolute yaw is not a stand-up requirement.  Wait only for yaw motion
    // to settle, then let NORMAL preserve/control the resulting heading.
    if (recover_balance_cnt_ >= k_recover_pitch_balance_hold &&
        fabsf(INS.YawSpeed) < k_recover_body_rate_ok) {
      if (recover_yaw_cnt_ < k_recover_pitch_yaw_hold)
        recover_yaw_cnt_++;
    } else if (recover_balance_cnt_ >= k_recover_pitch_balance_hold) {
      recover_yaw_cnt_ = 0;
    }
    break;
  }
  }
}

float balance_Chassis::GetJumpLandingWorldThetaRef() {
  return 0.5f * (jump_land_phi0_ref_l_ + jump_land_phi0_ref_r_) -
         0.5f * PI + INS.Pitch;
}

void balance_Chassis::JumpCalc() {
  const JumpSubStatus current_status = jump_status;
  JumpSubStatus next_status = current_status;
  const bool jump_left_contact_raw =
      left_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
  const bool jump_right_contact_raw =
      right_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
  if (current_status == JUMP_LAND_PREP) {
    if (jump_left_contact_raw) {
      jump_left_release_count_ = 0U;
      if (jump_left_contact_count_ < JUMP_LAND_CONTACT_CONFIRM_TICKS)
        jump_left_contact_count_++;
      if (jump_left_contact_count_ >= JUMP_LAND_CONTACT_CONFIRM_TICKS)
        jump_left_contact_ = true;
    } else {
      jump_left_contact_count_ = 0U;
      if (jump_left_contact_ &&
          jump_left_release_count_ < JUMP_LAND_CONTACT_RELEASE_TICKS)
        jump_left_release_count_++;
      if (jump_left_release_count_ >= JUMP_LAND_CONTACT_RELEASE_TICKS)
        jump_left_contact_ = false;
    }
    if (jump_right_contact_raw) {
      jump_right_release_count_ = 0U;
      if (jump_right_contact_count_ < JUMP_LAND_CONTACT_CONFIRM_TICKS)
        jump_right_contact_count_++;
      if (jump_right_contact_count_ >= JUMP_LAND_CONTACT_CONFIRM_TICKS)
        jump_right_contact_ = true;
    } else {
      jump_right_contact_count_ = 0U;
      if (jump_right_contact_ &&
          jump_right_release_count_ < JUMP_LAND_CONTACT_RELEASE_TICKS)
        jump_right_release_count_++;
      if (jump_right_release_count_ >= JUMP_LAND_CONTACT_RELEASE_TICKS)
        jump_right_contact_ = false;
    }
  }
  const bool jump_left_supported = jump_left_contact_;
  const bool jump_right_supported = jump_right_contact_;
  const bool jump_either_supported =
      jump_left_supported || jump_right_supported;
  const bool jump_both_supported =
      jump_left_supported && jump_right_supported;
  if (jump_both_supported && !jump_both_contact_latched_) {
    jump_both_contact_latched_ = true;
    jump_landing_brake_ref_l_ = left_wheel.Get_Now_Omega();
    jump_landing_brake_ref_r_ = right_wheel.Get_Now_Omega();
    jump_wheel_integral_l_ = 0.0f;
    jump_wheel_integral_r_ = 0.0f;
  }

  float left_ref = MIN_LEG_LENGTH;
  float right_ref = MIN_LEG_LENGTH;
  float left_ff = 0.0f;
  float right_ff = 0.0f;
  float direct_leg_force = 0.0f;
  bool use_direct_leg_force = false;
  bool use_landing_leg_force = false;

  if (current_status == JUMP_ASCEND || current_status == JUMP_RETRACT ||
      current_status == JUMP_LAND_PREP) {
    jump_timer++;
    const bool jump_raw_unloaded =
        (left_leg_.GetForceNormal() < OFF_GROUND_ENTER_THRESHOLD) &&
        (right_leg_.GetForceNormal() < OFF_GROUND_ENTER_THRESHOLD);
    if (GetOffGround() || jump_raw_unloaded) {
      jump_liftoff_seen_ = true;
    }
  }

  switch (current_status) {
  case JUMP_COMPRESS:
    // 阶段1：压缩蓄力。腿长压到最短并保持重力前馈，直到跳跃开关
    // 出现 2 -> 1 下降沿；跳跃专用轮速和姿态闭环已从本阶段开始生效。
    left_ref = JUMP_COMPRESS_LENGTH;
    right_ref = JUMP_COMPRESS_LENGTH;
    left_ff = k_gravity_comp * arm_cos_f32(left_leg_.GetTheta());
    right_ff = k_gravity_comp * arm_cos_f32(right_leg_.GetTheta());
    break;

  case JUMP_ASCEND:
    // 阶段2：以完整起跳轴向力伸腿；轮子不是零力矩自由滚动，而是闭环锁零。
    left_ref = MAX_LEG_LENGTH;
    right_ref = MAX_LEG_LENGTH;
    direct_leg_force = k_jump_force;
    use_direct_leg_force = true;

    if (left_leg_.GetLegLen() >=
            (MAX_LEG_LENGTH - JUMP_EXTEND_LENGTH_TOLERANCE) &&
        right_leg_.GetLegLen() >=
            (MAX_LEG_LENGTH - JUMP_EXTEND_LENGTH_TOLERANCE)) {
      if (jump_length_ready_count_ < JUMP_LENGTH_READY_TICKS) {
        jump_length_ready_count_++;
      }
    } else {
      jump_length_ready_count_ = 0;
    }
    // 伸腿阶段不得按固定时间提前结束。只有双腿都到达最大长度并连续
    // 确认后才进入收腿；若机构卡滞则保持满力伸腿，由急停负责中断。
    if (jump_length_ready_count_ >= JUMP_LENGTH_READY_TICKS) {
      next_status = JUMP_RETRACT;
    }
    break;

  case JUMP_RETRACT: {
    // 阶段3：直接施加收腿力，轮速和腿摆角继续锁定。
    left_ref = JUMP_RETRACT_LENGTH;
    right_ref = JUMP_RETRACT_LENGTH;

    if (left_leg_.GetLegLen() <=
            (JUMP_RETRACT_LENGTH + JUMP_RETRACT_LENGTH_TOLERANCE) &&
        right_leg_.GetLegLen() <=
            (JUMP_RETRACT_LENGTH + JUMP_RETRACT_LENGTH_TOLERANCE)) {
      if (jump_length_ready_count_ < JUMP_LENGTH_READY_TICKS) {
        jump_length_ready_count_++;
      }
      if (jump_length_ready_count_ >= JUMP_LENGTH_READY_TICKS &&
          jump_retract_hold_count_ < JUMP_RETRACT_AT_LENGTH_HOLD_TICKS) {
        jump_retract_hold_count_++;
      }
    } else {
      jump_length_ready_count_ = 0;
      jump_retract_hold_count_ = 0;
      // Use the larger force through the long airborne stroke, then soften
      // the final 5 cm so the mechanism reaches 0.20 m quickly without
      // slamming into the compact-length target.
      const float longest_leg =
          left_leg_.GetLegLen() > right_leg_.GetLegLen()
              ? left_leg_.GetLegLen()
              : right_leg_.GetLegLen();
      direct_leg_force = longest_leg > k_retract_near_length
                             ? k_retract_fast_force
                             : k_retract_near_force;
      use_direct_leg_force = true;
    }

    // Do not wait for ground contact at minimum leg length: after a short
    // compact-flight interval, deploy the legs before the first impact.
    const bool unexpected_early_contact =
        (left_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD) ||
        (right_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD);
    if ((jump_retract_hold_count_ >= JUMP_RETRACT_AT_LENGTH_HOLD_TICKS &&
         jump_liftoff_seen_) ||
        unexpected_early_contact) {
      next_status = JUMP_LAND_PREP;
    }
    break;
  }

  case JUMP_LAND_PREP: {
    // Extend progressively to a protected landing geometry. A bounded virtual
    // spring-damper absorbs compression instead of commanding the 220 N launch
    // force or allowing the chassis structure to touch down on folded legs.
    const float landing_ref = ClampRange(
        JUMP_RETRACT_LENGTH +
            JUMP_LAND_PREP_SLEW_PER_TICK * (float)jump_timer,
        JUMP_RETRACT_LENGTH, JUMP_LAND_PREP_LENGTH);
    left_ref = landing_ref;
    right_ref = landing_ref;
    left_leg_F_ = ClampRange(
        JUMP_LAND_PREP_KP * (left_ref - left_leg_.GetLegLen()) -
            JUMP_LAND_PREP_KD * left_leg_.GetLegSpeed(),
        JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    right_leg_F_ = ClampRange(
        JUMP_LAND_PREP_KP * (right_ref - right_leg_.GetLegLen()) -
            JUMP_LAND_PREP_KD * right_leg_.GetLegSpeed(),
        JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    use_landing_leg_force = true;

    // A stair-edge impact on only one wheel is not a stable landing. Require
    // both legs to carry load continuously after the global contact debounce.
    const float landing_chassis_speed =
        0.5f * (right_wheel.Get_Now_Omega() -
                left_wheel.Get_Now_Omega());
    const bool landing_speed_ready =
        fabsf(landing_chassis_speed) <= JUMP_LAND_EXIT_CHASSIS_SPEED;
    if (jump_both_contact_latched_ && landing_speed_ready) {
      if (jump_landing_ready_count_ < JUMP_LANDING_CONFIRM_TICKS) {
        jump_landing_ready_count_++;
      }
    } else {
      jump_landing_ready_count_ = 0;
    }
    if (jump_liftoff_seen_ &&
        jump_landing_ready_count_ >= JUMP_LANDING_CONFIRM_TICKS) {
      next_status = JUMP_NONE;
    }
    break;
  }

  case JUMP_NONE:
  default:
    // 理论上不会长期停留在这里；
    // 上层状态机会在下一拍把机器人切回 NORMAL。
    break;
  }

  // LQR retains body-state feedback; jump-only channels then override leg
  // swing angle and wheel speed as required by the current jump sub-state.
  active_left_leg_ref_ = left_ref;
  active_right_leg_ref_ = right_ref;
  LQRCalc();
  SynthesizeMotion();
  if (use_direct_leg_force) {
    // 本工程腿长是力控制而不是速度控制；直接使用完整轴向力等价于取消
    // 腿长 PID 输出滤波/斜坡，以最快可用速度伸腿或收腿。
    left_leg_F_ = direct_leg_force;
    right_leg_F_ = direct_leg_force;
  } else if (!use_landing_leg_force) {
    LegLenCalc(left_ref, right_ref, left_ff, right_ff);
  }

  if (current_status != JUMP_NONE) {
    float jump_dt = controller_dt_;
    if (jump_dt <= 0.0f || jump_dt > 0.02f) {
      jump_dt = 0.001f;
    }

    // Jump-only attitude controller. The common hip channel locks body pitch
    // to zero; the differential axial-force channel locks body roll to zero.
    // These states are reset whenever STATE_JUMPING is entered or exited.
    // Once airborne, integral torque cannot remove a persistent attitude error
    // and instead tends to reverse the body sharply when retract starts. Keep
    // integral action on the ground, then bleed it away and use rate damping.
    if (jump_liftoff_seen_) {
      jump_pitch_integral_ *= 0.85f;
    } else {
      jump_pitch_integral_ = ClampAbs(
          jump_pitch_integral_ + JUMP_PITCH_KI * INS.Pitch * jump_dt,
          JUMP_PITCH_INTEGRAL_MAX);
    }
    jump_roll_integral_ = ClampAbs(
        jump_roll_integral_ - JUMP_ROLL_KI * INS.Roll * jump_dt,
        JUMP_ROLL_INTEGRAL_MAX);
    const bool retract_pitch_control = current_status == JUMP_RETRACT;
    const float jump_pitch_kp =
        retract_pitch_control ? JUMP_RETRACT_PITCH_KP : JUMP_PITCH_KP;
    const float jump_pitch_kd =
        retract_pitch_control
            ? JUMP_RETRACT_PITCH_KD
            : (jump_liftoff_seen_ ? JUMP_AIR_PITCH_KD : JUMP_PITCH_KD);
    const float jump_pitch_torque = ClampAbs(
        jump_pitch_kp * INS.Pitch + jump_pitch_kd * INS.Gyro[1] +
            jump_pitch_integral_,
        JUMP_PITCH_TORQUE_MAX);
    const float jump_roll_force = ClampAbs(
        -JUMP_ROLL_KP * INS.Roll - JUMP_ROLL_KD * INS.Gyro[0] +
            jump_roll_integral_,
        JUMP_ROLL_FORCE_MAX);
    left_leg_F_ += jump_roll_force;
    right_leg_F_ -= jump_roll_force;

    // During launch/retract, phi0 remains a body-frame mechanism target.  In
    // LAND_PREP the requirement is different: the leg must be vertical in the
    // world/ground frame even if body pitch has not yet returned to zero.
    // VMC defines theta = phi0 - pi/2 + body_pitch, hence theta=0 requires
    // phi0_ref = pi/2 - body_pitch.
    const bool landing_world_vertical = current_status == JUMP_LAND_PREP;
    const float world_vertical_phi0_target = JUMP_PHI0_TARGET - INS.Pitch;
    if (landing_world_vertical) {
      const float left_ref_error = atan2f(
          arm_sin_f32(world_vertical_phi0_target - jump_land_phi0_ref_l_),
          arm_cos_f32(world_vertical_phi0_target - jump_land_phi0_ref_l_));
      const float right_ref_error = atan2f(
          arm_sin_f32(world_vertical_phi0_target - jump_land_phi0_ref_r_),
          arm_cos_f32(world_vertical_phi0_target - jump_land_phi0_ref_r_));
      jump_land_phi0_ref_l_ +=
          ClampAbs(left_ref_error, JUMP_LAND_PHI0_REF_SLEW_PER_TICK);
      jump_land_phi0_ref_r_ +=
          ClampAbs(right_ref_error, JUMP_LAND_PHI0_REF_SLEW_PER_TICK);
    }
    const float left_phi0_target =
        landing_world_vertical ? jump_land_phi0_ref_l_ : JUMP_PHI0_TARGET;
    const float right_phi0_target =
        landing_world_vertical ? jump_land_phi0_ref_r_ : JUMP_PHI0_TARGET;
    const float left_phi0_error_raw =
        left_phi0_target - left_leg_.GetPhi0();
    const float right_phi0_error_raw =
        right_phi0_target - right_leg_.GetPhi0();
    const float left_phi0_error =
        atan2f(arm_sin_f32(left_phi0_error_raw),
               arm_cos_f32(left_phi0_error_raw));
    const float right_phi0_error =
        atan2f(arm_sin_f32(right_phi0_error_raw),
               arm_cos_f32(right_phi0_error_raw));
    // Damping must use the derivative of the same controlled coordinate.
    // During landing that is world-frame theta_dot = phi0_dot + pitch_rate.
    const float left_phi0_rate =
        left_leg_.GetPhi0Speed() +
        (landing_world_vertical ? INS.Gyro[1] : 0.0f);
    const float right_phi0_rate =
        right_leg_.GetPhi0Speed() +
        (landing_world_vertical ? INS.Gyro[1] : 0.0f);
    // Do not superimpose the body-pitch hip torque in LAND_PREP: it acts on
    // the same common hip coordinate and would bias both legs away from the
    // world-vertical reference.  The existing wheel-reference trim retains
    // landing pitch authority without corrupting leg angle.
    const float hip_pitch_torque =
        landing_world_vertical ? 0.0f : jump_pitch_torque;
    const float land_phi0_kp = jump_either_supported
                                   ? JUMP_LAND_CONTACT_PHI0_KP
                                   : JUMP_LAND_PHI0_KP;
    const float land_phi0_kd = jump_either_supported
                                   ? JUMP_LAND_CONTACT_PHI0_KD
                                   : JUMP_LAND_PHI0_KD;
    const float phi0_kp = landing_world_vertical ? land_phi0_kp : JUMP_PHI0_KP;
    const float phi0_kd = landing_world_vertical ? land_phi0_kd : JUMP_PHI0_KD;
    const float hip_torque_limit = landing_world_vertical
                                       ? (jump_either_supported
                                              ? JUMP_LAND_CONTACT_PHI0_TORQUE_MAX
                                              : JUMP_LAND_PHI0_TORQUE_MAX)
                                       : (JUMP_PHI0_TORQUE_MAX +
                                          JUMP_PITCH_TORQUE_MAX);
    left_leg_T_ = ClampAbs(
        phi0_kp * left_phi0_error - phi0_kd * left_phi0_rate +
            hip_pitch_torque,
        hip_torque_limit);
    right_leg_T_ = ClampAbs(
        phi0_kp * right_phi0_error - phi0_kd * right_phi0_rate +
            hip_pitch_torque,
        hip_torque_limit);

  }

  if (current_status == JUMP_ASCEND || current_status == JUMP_RETRACT ||
      current_status == JUMP_LAND_PREP) {
    float jump_dt = controller_dt_;
    if (jump_dt <= 0.0f || jump_dt > 0.02f) {
      jump_dt = 0.001f;
    }
    // Jump-only motor-shaft speed PI. Hold the two full measured speeds
    // captured on the remote 2 -> 1 edge throughout every airborne phase.
    // Braking the wheels to zero at retract produced a large reaction torque
    // and the measured rapid rearward pitch. NORMAL keeps its own controller.
    const float left_omega = left_wheel.Get_Now_Omega();
    const float right_omega = right_wheel.Get_Now_Omega();
    // Preserve the captured translational wheel speed, but give the airborne
    // pitch loop a small common reference trim. Negative pitch is the measured
    // rearward body rotation, so it requests positive wheel acceleration and
    // the corresponding forward reaction torque on the chassis.
    const bool retract_wheel_pitch_control = current_status == JUMP_RETRACT;
    const float wheel_pitch_ref_kp =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_KP
                                    : JUMP_WHEEL_PITCH_REF_KP;
    const float wheel_pitch_ref_kd =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_KD
                                    : JUMP_WHEEL_PITCH_REF_KD;
    const float wheel_pitch_ref_max =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_MAX
                                    : JUMP_WHEEL_PITCH_REF_MAX;
    float pitch_speed_ref_trim = ClampAbs(
        -wheel_pitch_ref_kp * INS.Pitch -
            wheel_pitch_ref_kd * INS.Gyro[1],
        wheel_pitch_ref_max);
    float captured_speed_scale = 1.0f;
    float wheel_torque_limit = JUMP_WHEEL_TORQUE_MAX;
    if (current_status == JUMP_LAND_PREP) {
      // LAND_PREP never accelerates above the captured 2 -> 1 wheel speed.
      // Late in flight, pre-spin the wheels down modestly; raw first contact
      // starts a light brake immediately, while debounced contact permits a
      // stronger reduction.  Full braking still waits for bilateral support.
      float landing_speed_scale_target = 1.0f;
      float landing_speed_slew = JUMP_LAND_FLIGHT_SPEED_SLEW;
      if (jump_both_contact_latched_) {
        landing_speed_scale_target = 0.0f;
        landing_speed_slew = JUMP_LAND_BOTH_SPEED_SLEW;
      } else if (jump_either_supported) {
        landing_speed_scale_target = JUMP_LAND_SINGLE_CONTACT_SPEED_SCALE;
        landing_speed_slew = JUMP_LAND_SINGLE_SPEED_SLEW;
      } else if (jump_left_contact_raw || jump_right_contact_raw) {
        landing_speed_scale_target = JUMP_LAND_RAW_CONTACT_SPEED_SCALE;
        landing_speed_slew = JUMP_LAND_RAW_CONTACT_SPEED_SLEW;
      } else if (jump_timer >= JUMP_LAND_FLIGHT_DECEL_START_TICKS) {
        landing_speed_scale_target = JUMP_LAND_FLIGHT_SPEED_SCALE;
      }
      jump_landing_speed_scale_ += ClampRange(
          landing_speed_scale_target - jump_landing_speed_scale_,
          -landing_speed_slew, landing_speed_slew);
      captured_speed_scale = jump_landing_speed_scale_;
      if (jump_either_supported) {
        // Once a wheel touches, prioritise predictable forward placement over
        // an aggressive pitch trim that could reverse the approach reference.
        pitch_speed_ref_trim *= 0.5f;
      }
      wheel_torque_limit = JUMP_LAND_WHEEL_TORQUE_MAX;
      if (!jump_either_supported &&
          (jump_left_contact_raw || jump_right_contact_raw)) {
        wheel_torque_limit = JUMP_LAND_RAW_CONTACT_TORQUE_MAX;
        jump_wheel_integral_l_ *= 0.85f;
        jump_wheel_integral_r_ *= 0.85f;
      }
    }
    float left_ref = captured_speed_scale * jump_wheel_speed_ref_l_ +
                     pitch_speed_ref_trim;
    float right_ref = captured_speed_scale * jump_wheel_speed_ref_r_ +
                      pitch_speed_ref_trim;
    if (current_status == JUMP_LAND_PREP && jump_both_contact_latched_) {
      // Start from the measured speeds at confirmed contact, then slew each
      // reference to zero.  This removes the touchdown reference step.
      jump_landing_brake_ref_l_ += ClampRange(
          -jump_landing_brake_ref_l_, -JUMP_LAND_BRAKE_REF_SLEW,
          JUMP_LAND_BRAKE_REF_SLEW);
      jump_landing_brake_ref_r_ += ClampRange(
          -jump_landing_brake_ref_r_, -JUMP_LAND_BRAKE_REF_SLEW,
          JUMP_LAND_BRAKE_REF_SLEW);
      // Keep a reduced differential-reaction pitch channel while the
      // translational references slew to zero.  This prevents stopping
      // distance improvements from sacrificing touchdown attitude control.
      left_ref = jump_landing_brake_ref_l_ +
                 JUMP_LAND_BRAKE_PITCH_TRIM_SCALE * pitch_speed_ref_trim;
      right_ref = jump_landing_brake_ref_r_ +
                  JUMP_LAND_BRAKE_PITCH_TRIM_SCALE * pitch_speed_ref_trim;

      // Once a wheel has crossed zero relative to its touchdown direction,
      // stop asking the speed loop for further translational braking on that
      // side. Track the measured speed and retain only the reduced pitch trim.
      const bool left_zero_crossed_now =
          fabsf(jump_wheel_speed_ref_l_) >
              JUMP_LAND_BRAKE_ZERO_CROSS_SPEED &&
          left_omega * jump_wheel_speed_ref_l_ <= 0.0f;
      const bool right_zero_crossed_now =
          fabsf(jump_wheel_speed_ref_r_) >
              JUMP_LAND_BRAKE_ZERO_CROSS_SPEED &&
          right_omega * jump_wheel_speed_ref_r_ <= 0.0f;
      jump_landing_zero_cross_l_ =
          jump_landing_zero_cross_l_ || left_zero_crossed_now;
      jump_landing_zero_cross_r_ =
          jump_landing_zero_cross_r_ || right_zero_crossed_now;
      if (jump_landing_zero_cross_l_) {
        jump_landing_brake_ref_l_ = 0.0f;
        left_ref = left_omega +
                   JUMP_LAND_BRAKE_PITCH_TRIM_SCALE * pitch_speed_ref_trim;
      }
      if (jump_landing_zero_cross_r_) {
        jump_landing_brake_ref_r_ = 0.0f;
        right_ref = right_omega +
                    JUMP_LAND_BRAKE_PITCH_TRIM_SCALE * pitch_speed_ref_trim;
      }
    }
    if (current_status == JUMP_LAND_PREP) {
      // Pitch correction may reduce the captured reference, but must not turn
      // it into a command in the opposite travel direction.
      if (jump_wheel_speed_ref_l_ > 0.0f && left_ref < 0.0f)
        left_ref = 0.0f;
      if (jump_wheel_speed_ref_l_ < 0.0f && left_ref > 0.0f)
        left_ref = 0.0f;
      if (jump_wheel_speed_ref_r_ > 0.0f && right_ref < 0.0f)
        right_ref = 0.0f;
      if (jump_wheel_speed_ref_r_ < 0.0f && right_ref > 0.0f)
        right_ref = 0.0f;
    }
    jump_active_wheel_ref_l_ = left_ref;
    jump_active_wheel_ref_r_ = right_ref;
    const float left_speed_error = left_omega - left_ref;
    const float right_speed_error = right_omega - right_ref;

    // SetMotorTor() 会对左轮命令取反，因此两侧内部力矩符号不同；最终发给
    // 两个电机的都是与各自实测转速相反的制动力矩。
    if (current_status == JUMP_LAND_PREP && jump_both_contact_latched_) {
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      l_wheel_T_ = ClampAbs(JUMP_LAND_BRAKE_SPEED_KP * left_speed_error,
                            JUMP_LAND_BRAKE_TORQUE_MAX);
      r_wheel_T_ = ClampAbs(-JUMP_LAND_BRAKE_SPEED_KP * right_speed_error,
                            JUMP_LAND_BRAKE_TORQUE_MAX);
    } else {
      jump_wheel_integral_l_ = ClampAbs(
          jump_wheel_integral_l_ +
              JUMP_WHEEL_SPEED_KI * left_speed_error * jump_dt,
          JUMP_WHEEL_INTEGRAL_MAX);
      jump_wheel_integral_r_ = ClampAbs(
          jump_wheel_integral_r_ -
              JUMP_WHEEL_SPEED_KI * right_speed_error * jump_dt,
          JUMP_WHEEL_INTEGRAL_MAX);
      l_wheel_T_ = ClampAbs(JUMP_WHEEL_SPEED_KP * left_speed_error +
                                jump_wheel_integral_l_,
                            wheel_torque_limit);
      r_wheel_T_ = ClampAbs(-JUMP_WHEEL_SPEED_KP * right_speed_error +
                                jump_wheel_integral_r_,
                            wheel_torque_limit);
    }
  }

  TorCalc();

  // 本拍输出全部生成完之后，再切换到下一子状态。
  // 这样下一拍开始时，新的子状态才真正生效，逻辑更清晰。
  if (next_status != current_status) {
    jump_status = next_status;
    jump_timer = 0;
    jump_length_ready_count_ = 0;
    jump_retract_hold_count_ = 0;
    jump_landing_ready_count_ = 0;
    // 子状态之间保留跳跃专用积分，退出跳跃时统一清零。
    if (next_status == JUMP_RETRACT) {
      // Remove the grounded extension bias before the airborne retract impulse.
      jump_pitch_integral_ = 0.0f;
    }
    if (next_status == JUMP_LAND_PREP) {
      // Initialise the world-frame landing references from the current leg
      // angles so pitch compensation cannot create an entry step.
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      jump_landing_speed_scale_ = 1.0f;
      jump_landing_zero_cross_l_ = false;
      jump_landing_zero_cross_r_ = false;
      jump_land_phi0_ref_l_ = left_leg_.GetPhi0();
      jump_land_phi0_ref_r_ = right_leg_.GetPhi0();
    }
    if (next_status == JUMP_NONE) {
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      jump_pitch_integral_ = 0.0f;
      jump_roll_integral_ = 0.0f;
    }
  }
}

/**
 * @brief Joint debug IK position control
 */
void balance_Chassis::JointDebugCalc() {
  // joint debug 只在这里计算目标，不直接下发电机命令。
  joint_debug_output_enable_ = false;
  joint_debug_vel_limit_ = 0.0f;
  joint_debug_lf_target_ = 0.0f;
  joint_debug_lb_target_ = 0.0f;
  joint_debug_rf_target_ = 0.0f;
  joint_debug_rb_target_ = 0.0f;
  l_wheel_T_ = 0.0f;
  r_wheel_T_ = 0.0f;

  const float target_phi0 = CHASSIS_JOINT_DEBUG_PHI0;
  const float target_l0 = CHASSIS_JOINT_DEBUG_L0;

  left_leg_.SetLegIKTarget(target_phi0, target_l0);
  right_leg_.SetLegIKTarget(target_phi0, target_l0);
  left_leg_.LegInverseCalc();
  right_leg_.LegInverseCalc();

  if (!left_leg_.GetIKValid() || !right_leg_.GetIKValid()) {
    return;
  }

  const float lb_target = left_leg_.GetPhi4IK();
  const float lf_target = left_leg_.GetPhi1IK();
  const float rb_target = -right_leg_.GetPhi4IK();
  const float rf_target = -right_leg_.GetPhi1IK();

  joint_debug_lf_target_ = lf_target;
  joint_debug_lb_target_ = lb_target;
  joint_debug_rf_target_ = rf_target;
  joint_debug_rb_target_ = rb_target;
  joint_debug_vel_limit_ = 0.5f;
  joint_debug_output_enable_ = true;

  // status_flag == 1 时允许轮子参与平衡；status_flag == 2 时只调关节。
  if (sbus_rx_data.status_flag == 1) {
    LQRCalc();
    l_wheel_T_ = lqr_body_.GetWheelTorL();
    r_wheel_T_ = lqr_body_.GetWheelTorR();
  }
}

/**
 * @brief LQR calculation
 */
void balance_Chassis::LQRCalc() {
  const bool debug_mode = (robot_status == STATE_JOINT_DEBUG);
  const bool recover_mode = (robot_status == STATE_RECOVERING);
  const bool recover_center_mode =
      recover_mode && (recover_sub_status_ == RECOVER_CENTER_LEGS);
  const bool recover_balance_mode =
      recover_mode && (recover_sub_status_ == RECOVER_BALANCE);
  const bool normal_yaw_external = (robot_status == STATE_NORMAL);
  const bool normal_mode = (robot_status == STATE_NORMAL);
  const bool normal_translation_active =
      normal_mode && (fabsf(target_speed_) > kTranslationCommandDeadband);
  const bool normal_pivot = normal_mode && !normal_translation_active &&
                            (fabsf(target_w_rotation_) > kYawCommandDeadband);
  // A NORMAL pivot keeps the ordinary NORMAL LQR speed target and measured
  // centre speed.  Yaw is external only in the differential coordinate; the
  // LQR common-speed and pitch balance states remain unchanged.
  lqr_body_.SetSpeed(normal_mode ? normal_speed_ref_ : target_speed_);
  // The calibrated NORMAL mechanical zero is fixed at 0.050 rad. Do not add a
  // yaw/translation pitch feedforward: hardware data shows it as a persistent
  // I0 offset while crossing terrain.
  const float pivot_pitch_target_desired =
      normal_mode ? kNormalPitchZeroOffset : 0.0f;
  const float pitch_slew_step =
      kPivotPitchFfSlew * ((controller_dt_ > 0.0f && controller_dt_ < 0.02f)
                               ? controller_dt_
                               : 0.001f);
  if (normal_pivot) {
    // Keep the pivot pitch reference exactly zero.  Integrating this reference
    // changes the LQR common wheel torque and moved I2 to about -0.26 m/s even
    // though yaw differential control was correct.
    normal_pivot_pitch_target_ = kNormalPitchZeroOffset;
  } else {
    normal_pivot_pitch_target_ =
        SlewTowards(normal_pivot_pitch_target_, pivot_pitch_target_desired,
                    pitch_slew_step);
  }
  lqr_body_.SetPitchTarget(normal_mode ? normal_pivot_pitch_target_ : 0.0f);
  // NORMAL uses a small standstill bias to compensate the measured zero-input
  // drift. A pivot still keeps it at zero because any common-speed bias moves
  // the rotation centre off the axle.
  const float normal_bias_scale =
      normal_translation_active
          ? ClampRange(fabsf(normal_speed_ref_) / kNormalSpeedBiasRampSpeed,
                       0.0f, 1.0f)
          : 0.0f;
  const float normal_speed_bias =
      normal_translation_active
          ? kNormalSpeedBias * normal_bias_scale
          : ((normal_mode && !normal_pivot) ? kNormalZeroInputSpeedBias : 0.0f);
  lqr_body_.SetSpeedBias(normal_speed_bias);
  lqr_body_.SetDist(target_dist_);
  // Pivot yaw is handled after LQR in a dedicated differential-wheel loop.
  // Zeroing only the NORMAL pivot yaw states here prevents the same yaw error
  // from also modulating common wheel and leg torques (pitch disturbance).
  lqr_body_.SetRotation(normal_yaw_external ? 0.0f : target_rotation_);
  lqr_body_.SetWRotation(normal_yaw_external ? 0.0f : target_w_rotation_);
  lqr_body_.SetPitchGainScale(
      normal_pivot ? kNormalPivotPitchGainScale
                   : (normal_mode ? kNormalDrivePitchGainScale : 1.0f));
  // debug模式下，用机身pitch和gyro代替腿部数据
  float theta_l = debug_mode ? INS.Pitch : left_leg_.GetTheta();
  float theta_r = debug_mode ? INS.Pitch : right_leg_.GetTheta();
  float w_theta_l = debug_mode ? INS.Gyro[1] : left_leg_.GetDotTheta();
  float w_theta_r = debug_mode ? INS.Gyro[1] : right_leg_.GetDotTheta();
  if (recover_center_mode) {
    // The leg PD, not wheel translation, must remove most of the remaining
    // phi0 error. Keep only a small leg-angle contribution for wheel capture;
    // body pitch and speed feedback retain their full LQR authority.
    theta_l *= k_recover_center_lqr_theta_scale;
    theta_r *= k_recover_center_lqr_theta_scale;
    w_theta_l *= k_recover_center_lqr_theta_scale;
    w_theta_r *= k_recover_center_lqr_theta_scale;
  } else if (recover_balance_mode) {
    float theta_blend = recover_sub_timer_ / k_recover_balance_wheel_ramp_ticks;
    if (theta_blend > 1.0f)
      theta_blend = 1.0f;
    const float theta_scale =
        k_recover_center_lqr_theta_scale +
        theta_blend * (1.0f - k_recover_center_lqr_theta_scale);
    theta_l *= theta_scale;
    theta_r *= theta_scale;
    w_theta_l *= theta_scale;
    w_theta_r *= theta_scale;
  }
  float lqr_leg_len_l = left_leg_.GetLegLen();
  float lqr_leg_len_r = right_leg_.GetLegLen();
  if (recover_mode) {
    // Recovery can call LQR before the legs enter its fitted length interval.
    // Clamp only the gain-scheduling input; VMC still uses the measured length.
    if (lqr_leg_len_l < MIN_LEG_LENGTH)
      lqr_leg_len_l = MIN_LEG_LENGTH;
    if (lqr_leg_len_l > MAX_LEG_LENGTH)
      lqr_leg_len_l = MAX_LEG_LENGTH;
    if (lqr_leg_len_r < MIN_LEG_LENGTH)
      lqr_leg_len_r = MIN_LEG_LENGTH;
    if (lqr_leg_len_r > MAX_LEG_LENGTH)
      lqr_leg_len_r = MAX_LEG_LENGTH;
  }
  // Recovery has a dedicated, bounded yaw loop. Feeding yaw rate into LQR as
  // well would apply the same correction twice and can spin a short robot.
  const float lqr_rotation =
      (recover_mode || normal_yaw_external) ? 0.0f : rotation_;
  const float lqr_w_rotation =
      (recover_mode || normal_yaw_external) ? 0.0f : INS.Gyro[2];
  // NORMAL pivots retain the same LQR speed state and speed error as normal
  // driving.  The additional midpoint loop only removes the common drift that
  // would otherwise make the two pivot wheel speeds unequal.
  const float lqr_speed =
      (robot_status == STATE_NORMAL) ? normal_wheel_center_speed_ : vel_;
  lqr_body_.SetData(dist_, lqr_speed, lqr_rotation, lqr_w_rotation, theta_l,
                    w_theta_l, theta_r, w_theta_r, INS.Pitch, INS.Gyro[1],
                    lqr_leg_len_l, left_leg_.GetForceNormal(), lqr_leg_len_r,
                    right_leg_.GetForceNormal(), GetOffGround());
  lqr_body_.Calc();
}

/**
 * @brief 控制力矩计算
 * @param
 * @param
 * @param
 */
void balance_Chassis::TorCalc() {
  left_leg_.SetTor(left_leg_F_, left_leg_T_);
  right_leg_.SetTor(right_leg_F_, right_leg_T_);
  left_leg_.TorCalc();
  right_leg_.TorCalc();
}

/**
 * @brief 力矩控制下发电机条件判断
 * @param
 * @param
 * @param
 */
void balance_Chassis::TorControl() {
  // 示教模式下停止电机输出
#if CHASSIS_TEACH_ENABLE
  StopMotor();
  return;
#endif
  // ESTOP： WarmingMotorControl 已在 ChassissControl 中处理电机输出
  if (robot_status == STATE_ESTOP) {
    return;
  }
  if (robot_status == STATE_JOINT_DEBUG) {
    SetJointDebugMotor();
    return;
  }
  SetMotorTor();
}

/**
 * @brief 腿部控制器计算
 * @param
 * @param
 * @param
 */
void balance_Chassis::LegLenCalc(float left_ref, float right_ref, float left_ff,
                                 float right_ff) {
  // 纯腿长控制公共函数：
  // 只根据调用者给定的参考值和前馈计算 left_leg_F_/right_leg_F_，
  // 不再内部判断当前机器人状态。
  left_leg_len_.SetMeasure(left_leg_.GetLegLen());
  right_leg_len_.SetMeasure(right_leg_.GetLegLen());
  left_leg_len_.SetRef(left_ref);
  right_leg_len_.SetRef(right_ref);
  left_leg_F_ = left_leg_len_.Calculate() + left_ff;
  right_leg_F_ = right_leg_len_.Calculate() + right_ff;
}

/**
 * @brief LQR 运动控制量整合
 * @param
 * @param
 * @param
 */
void balance_Chassis::SynthesizeMotion() {

  l_wheel_T_ = lqr_body_.GetWheelTorL();
  r_wheel_T_ = lqr_body_.GetWheelTorR();
  left_leg_T_ = lqr_body_.GetLegTorL();
  right_leg_T_ = lqr_body_.GetLegTorR();
}

/**
 * @brief 底盘观测任务
 * @param
 * @param
 * @param
 */
void balance_Chassis::Observe() {
  SpeedCalc();
  LegCalc();
  OffGroundDetect();
}

/**
 * @brief 离地标志判断函数
 * @param
 * @param
 * @param
 */
void balance_Chassis::OffGroundDetect() {
  // 只有开启离地检测且遥控器允许机器人工作时，才判断离地状态
  if (!OFF_GROUND_DETECT_ENABLE) {
    off_ground_ = false;
    off_ground_enter_count_ = 0;
    off_ground_exit_count_ = 0;
    return;
  }
  if (sbus_rx_data.status_flag == 3 || sbus_rx_data.status_flag == 0) {
    off_ground_ = false;
    off_ground_enter_count_ = 0;
    off_ground_exit_count_ = 0;
    return;
  }

  const float left_normal_force = left_leg_.GetForceNormal();
  const float right_normal_force = right_leg_.GetForceNormal();
  if (!off_ground_) {
    off_ground_exit_count_ = 0;
    const bool both_unloaded =
        (left_normal_force < OFF_GROUND_ENTER_THRESHOLD) &&
        (right_normal_force < OFF_GROUND_ENTER_THRESHOLD);
    if (both_unloaded) {
      if (off_ground_enter_count_ < OFF_GROUND_ENTER_TICKS)
        off_ground_enter_count_++;
      if (off_ground_enter_count_ >= OFF_GROUND_ENTER_TICKS) {
        off_ground_ = true;
        off_ground_enter_count_ = 0;
      }
    } else {
      off_ground_enter_count_ = 0;
    }
  } else {
    off_ground_enter_count_ = 0;
    const bool support_recovered =
        (left_normal_force > OFF_GROUND_EXIT_THRESHOLD) ||
        (right_normal_force > OFF_GROUND_EXIT_THRESHOLD);
    if (support_recovered) {
      if (off_ground_exit_count_ < OFF_GROUND_EXIT_TICKS)
        off_ground_exit_count_++;
      if (off_ground_exit_count_ >= OFF_GROUND_EXIT_TICKS) {
        off_ground_ = false;
        off_ground_exit_count_ = 0;
      }
    } else {
      off_ground_exit_count_ = 0;
    }
  }
}
/**
 * @brief 力矩指令设置
 * @param
 * @param
 * @param
 */
// 电机力矩输入模式
void balance_Chassis::SetMotorTor() {

  left_wheel.Set_Target_Torque(-l_wheel_T_);
  right_wheel.Set_Target_Torque(r_wheel_T_);
  lb_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, left_leg_.GetT2());
  lf_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, left_leg_.GetT1());
  rb_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, -right_leg_.GetT2());
  rf_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, -right_leg_.GetT1());
}

void balance_Chassis::SetJointDebugMotor() {
  left_wheel.Set_Target_Torque(-l_wheel_T_);
  right_wheel.Set_Target_Torque(r_wheel_T_);

  if (!joint_debug_output_enable_) {
    lf_joint_.Set_PosVel(0.0f, 0.0f);
    lb_joint_.Set_PosVel(0.0f, 0.0f);
    rf_joint_.Set_PosVel(0.0f, 0.0f);
    rb_joint_.Set_PosVel(0.0f, 0.0f);
    return;
  }

  lf_joint_.Set_PosVel(joint_debug_lf_target_, joint_debug_vel_limit_);
  lb_joint_.Set_PosVel(joint_debug_lb_target_, joint_debug_vel_limit_);
  rf_joint_.Set_PosVel(joint_debug_rf_target_, joint_debug_vel_limit_);
  rb_joint_.Set_PosVel(joint_debug_rb_target_, joint_debug_vel_limit_);
}
/**
 * @brief 力矩指令设置 - 急停
 * @param
 * @param
 * @param
 */
// 电机急停模式
void balance_Chassis::StopMotor() {
  left_wheel.Set_Target_Torque(0.0f);
  right_wheel.Set_Target_Torque(0.0f);
  if (robot_status == STATE_JOINT_DEBUG) {
    lf_joint_.Set_PosVel(0.0f, 0.0f);
    lb_joint_.Set_PosVel(0.0f, 0.0f);
    rf_joint_.Set_PosVel(0.0f, 0.0f);
    rb_joint_.Set_PosVel(0.0f, 0.0f);
  } else {
    lf_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    lb_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    rf_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    rb_joint_.Set_MIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
}

/**
 * @brief 设置腿长
 * @param
 * @param
 * @param
 */
void balance_Chassis::SetLegLen() {
  // 腿长目标仅从遥控输入读取，不在这里做状态判断。
  target_len_ = sbus_rx_data.len;
  //  target_len_ =  0.23;
}

/**
 * @brief 设置YAW
 * @param
 * @param
 * @param
 */
// void balance_Chassis::SetFollow()
//{
// if (board_comm.rece_.AutoAimFlag == 1)
//{
// static float delta_aim_yaw = 0.0f;
// YAW.SetAutoAimFlag(board_comm.rece_.AutoAimFlag);
// delta_aim_yaw -= board_comm.rece_.yaw_speed * 0.006;
// YAW.SetTargetAngle(board_comm.rece_.yaw_position + delta_aim_yaw +
// Saber_Angle.Yaw/360.0f*8192); if (YAW.targetAngle > 8192)
//{
// YAW.targetAngle -= 8192;
// }
// else if (YAW.targetAngle < 0)
//{
// YAW.targetAngle += 8192;
// }
// float delta_yaw= YAW.targetAngle - YAW.realAngle;
// if(delta_yaw<-4096)
// delta_yaw+=8192;
// if(delta_yaw>4096)
// delta_yaw-=8192;
////M6020s_Yaw.targetSpeed = Position_PID(&M6020s_Yaw.Aim_position_PID, 0,
/// delta_yaw); /M6020s_Yaw.outCurrent =
/// Position_PID(&M6020s_Yaw.Aim_position_PID, M6020s_Yaw.realSpeed,
/// M6020s_Yaw.targetSpeed);
//}
// else if (board_comm.rece_.AutoAimFlag == 0)
//{
// YAW.SetAutoAimFlag(board_comm.rece_.AutoAimFlag);
// YAW.targetAngle -= board_comm.rece_.yaw_speed * 0.006f - 0*
// Saber_Angle.Gyro_z; if (YAW.targetAngle > 8192)
//{
// YAW.targetAngle -= 8192;
//}
// else if (YAW.targetAngle < 0)
//{
// YAW.targetAngle += 8192;
//}
// float delta_yaw= YAW.targetAngle - YAW.realAngle;
// if(delta_yaw<-4096)
// delta_yaw+=8192;
// if(delta_yaw>4096)
// delta_yaw-=8192;
// YAW.targetSpeed = Position_PID(&YAW.position_PID, 0, delta_yaw);
// YAW.outCurrent = Position_PID(&YAW.velocity_PID, YAW.realSpeed,
// YAW.targetSpeed);
//}
//}
/**
 * @brief 设置速度
 * @param
 * @param
 * @param
 */
void balance_Chassis::SetSpd() {
  // Read translation first: the extra yaw shaping below is intentionally
  // limited to an in-place NORMAL request.
  target_speed_ = -sbus_rx_data.speed;
  if (fabsf(target_speed_) <= kTranslationCommandDeadband) {
    target_speed_ = 0.0f;
  }

  float yaw_desired = sbus_rx_data.yaw_speed;
  bool yaw_stick_requested = fabsf(yaw_desired) > kYawCommandDeadband;
  if (robot_status == STATE_NORMAL) {
    // map_sbus_to_range() has already removed the dead zone and linearly
    // mapped the remaining stick travel to +/-YAW_SPEED_MAX.  Use that result
    // directly; do not normalise, rescale or ramp it again in the chassis.
    // Preserve the user's dead-zone linear mapping with a unit physical scale.
    // The previous extra x2 multiplier made the internal yaw request larger
    // than the speed entered by the remote controller.
    yaw_desired = sbus_rx_data.yaw_speed_mapped * kNormalYawRateScale;
    yaw_stick_requested = fabsf(yaw_desired) > kYawCommandDeadband;
    if (fabsf(target_speed_) <= kTranslationCommandDeadband) {
      float yaw_step = kNormalPivotYawRateAccel * 0.001f;
      if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
        yaw_step = kNormalPivotYawRateAccel * controller_dt_;
      }
      normal_pivot_yaw_rate_ref_ =
          SlewTowards(normal_pivot_yaw_rate_ref_, yaw_desired, yaw_step);
      target_w_rotation_ = normal_pivot_yaw_rate_ref_;
    } else {
      normal_pivot_yaw_rate_ref_ = yaw_desired;
      target_w_rotation_ = yaw_desired;
    }
  } else {
    normal_pivot_yaw_rate_ref_ = 0.0f;
    target_w_rotation_ = yaw_desired;
  }
  // 速度目标在状态层统一管理，这里只负责写入当前期望值。

  if (fabsf(target_speed_) <= kTranslationCommandDeadband)
    target_speed_ = 0.0f;

  // At the turn-to-centre transition discard every NORMAL-only pivot state
  // and latch the current pose.  No differential or common integral from the
  // previous turn may enter the next straight-line command.
  if (robot_status == STATE_NORMAL && !yaw_stick_requested &&
      yaw_command_active_) {
    target_w_rotation_ = 0.0f;
    normal_pivot_yaw_rate_ref_ = 0.0f;
    target_rotation_ = rotation_;
    target_dist_ = dist_;
    normal_pivot_yaw_torque_cmd_ = 0.0f;
    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_pitch_integral_ = 0.0f;
    normal_pivot_center_prev_speed_ = 0.0f;
    normal_pivot_leg_sync_force_ = 0.0f;
    normal_pivot_leg_sync_integral_ = 0.0f;
    normal_zero_speed_trim_torque_ = 0.0f;
    roll_force_cmd_ = 0.0f;
    normal_pivot_roll_integral_ = 0.0f;
    roll_comp_.Clear();
    yaw_command_active_ = false;
  }
  // The stable build did not accumulate a position-return state. Keep this
  // reference on the measured position so normal motion is governed by the
  // fitted speed and attitude feedback without stored displacement energy.
  target_dist_ = dist_;
  translation_command_active_ = false;

  // Use rate control while the operator is turning. On release, latch the
  // actual heading rather than the integrated command heading. This prevents
  // stored angle error from rotating the chassis back in the opposite
  // direction. Test the actual stick-derived request, not an intermediate slew
  // value. Testing the first slew step (< deadband) used to clear it every
  // cycle and made I9 remain permanently zero.
  const bool yaw_requested = yaw_stick_requested;
  // NORMAL does not hold or integrate absolute heading.  Refreshing this
  // reference every cycle guarantees that a completed turn cannot be paid
  // back as an opposite turn when the next translation command arrives.
  if (robot_status == STATE_NORMAL) {
    target_rotation_ = rotation_;
  }
  if (yaw_requested) {
    yaw_command_active_ = true;
  } else {
    target_w_rotation_ = 0.0f;
    if (yaw_command_active_) {
      target_rotation_ = rotation_;
      yaw_command_active_ = false;
    }
  }
}

/**
 * @brief 设置机器人状态
 * @param
 * @param ------------------------------
 * @param
 */
void balance_Chassis::SetState() {
  // 状态切换和控制目标刷新拆开处理，避免一个函数里混合多种职责。
  UpdateStateMachine();
  UpdateCommandByState();
}

float balance_Chassis::GetRecoverPhi0Ref() {
  return recover_state_ ? (0.5f * PI) : recover_target_phi0;
}

/**
 * @brief 速度计算
 * @param
 * @param
 * @param
 */
void balance_Chassis::SpeedCalc() {
  //  left_w_wheel_ = M3508_Array[0].realSpeed/60*2*3.14159+
  //  left_leg_.GetPhi2Speed() - INS.Gyro[0];
  // right_w_wheel_ =-M3508_Array[1].realSpeed/60*2*3.14159+
  // right_leg_.GetPhi2Speed() - INS.Gyro[0];

  // left_w_wheel_ = -left_wheel.Get_Now_Omega() - left_leg_.GetPhi0Speed() +
  // INS.Gyro[1]; right_w_wheel_ = right_wheel.Get_Now_Omega() -
  // right_leg_.GetPhi0Speed() + INS.Gyro[1];
  left_w_wheel_ =
      -left_wheel.Get_Now_Omega() + left_leg_.GetPhi0Speed() + INS.Gyro[1];
  right_w_wheel_ =
      right_wheel.Get_Now_Omega() + right_leg_.GetPhi0Speed() + INS.Gyro[1];
  left_v_body_ = left_w_wheel_ * k_wheel_radius;
  right_v_body_ = right_w_wheel_ * k_wheel_radius;
  // left_v_body_ = left_w_wheel_ * k_wheel_radius +
  //               left_leg_.GetLegLen() *
  //               left_leg_.GetDotTheta()*arm_cos_f32(left_leg_.GetTheta()) +
  //               left_leg_.GetLegSpeed() * arm_sin_f32(left_leg_.GetTheta());
  // right_v_body_ = right_w_wheel_ * k_wheel_radius +
  //                right_leg_.GetLegLen() *
  //                right_leg_.GetDotTheta()*arm_cos_f32(right_leg_.GetTheta())
  //                + right_leg_.GetLegSpeed() *
  //                arm_sin_f32(right_leg_.GetTheta());

  vel_m = (left_v_body_ + right_v_body_) / 2;
  //    if (left_leg_.GetForceNormal() < 20.0f &&right_leg_.GetForceNormal()
  //    < 20.0f) {
  // vel_m = 0;
  //  }
  controller_dt_ = DWT_GetDeltaT(&dwt_cnt_controller_);

  // Unwrap IMU yaw into the LQR rotation state.  INS.Yaw may cross +/-pi (or
  // 0/2pi); wrapping the increment prevents a full-turn error at that point.
  if (!heading_initialized_) {
    heading_last_ = INS.Yaw;
    rotation_ = 0.0f;
    target_rotation_ = 0.0f;
    heading_initialized_ = true;
  } else {
    float heading_delta = INS.Yaw - heading_last_;
    if (heading_delta > PI)
      heading_delta -= 2.0f * PI;
    if (heading_delta < -PI)
      heading_delta += 2.0f * PI;
    rotation_ += heading_delta;
    heading_last_ = INS.Yaw;
  }

  //  使用 kf 同时估计加速度和速度，进行滤波更新
  kf.MeasuredVector[0] = vel_m;
  kf.MeasuredVector[1] = INS.MotionAccel_n[0];
  kf.F_data[1] = controller_dt_; // 更新F矩阵
  Kalman_Filter_Update(&kf);
  vel_ = kf.xhat_data[0];
  acc_ = kf.xhat_data[1];
  if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
    dist_m_ += vel_ * controller_dt_;
  }
  dist_ = dist_m_;
  // vel_ = vel_m;
}
/**
 * @brief 对必要的观测量进行滤波
 * @param
 * @param
 * @param
 */
void balance_Chassis::Filter(float dtheta_b_, float dphi_) {
  dtheta_b_filter_ = 0.8 * dtheta_b_filter_ + 0.2 * dtheta_b_;
  dphi_filter_ = 0.8 * dphi_filter_ + 0.2 * dphi_;
  if (fabs(dtheta_b_filter_) < 0.005)
    dtheta_b_filter_ = 0;
  if (fabs(dphi_filter_) < 0.005)
    dphi_filter_ = 0;
}

void balance_Chassis::getangle(float angle) { Angle_ChassisToCloud = angle; }
// 实现C接口函数
extern "C" {
void chassis_lf_joint_getInfo(FDCan_Export_Data_t data) {
  chassis.lf_joint_.DM_8009P_getInfo(data);
}

void chassis_lb_joint_getInfo(FDCan_Export_Data_t data) {
  chassis.lb_joint_.DM_8009P_getInfo(data);
}

void chassis_rf_joint_getInfo(FDCan_Export_Data_t data) {
  chassis.rf_joint_.DM_8009P_getInfo(data);
}

void chassis_rb_joint_getInfo(FDCan_Export_Data_t data) {
  chassis.rb_joint_.DM_8009P_getInfo(data);
}
}
