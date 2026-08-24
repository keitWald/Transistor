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
const float k_retract_fast_force = -250.0f;
const float k_retract_near_force = -200.0f;
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

float CalculateLandingImpedanceForce(
    float length_ref, float length, float length_speed, float gravity_ff,
    float kp, float kd, float force_min, float force_max, float soft_limit,
    float soft_stop_kp, float soft_stop_kd, bool contact) {
  float force = kp * (length_ref - length) - kd * length_speed + gravity_ff;
  if (contact && length < soft_limit && length_speed < 0.0f) {
    force += soft_stop_kp * (soft_limit - length) -
             soft_stop_kd * length_speed;
  }
  return ClampRange(force, force_min, force_max);
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
  EnableAllJointMotors();
}
/**
 * @brief 底盘状态初始化
 * @param
 * @param
 * @param
 */
void balance_Chassis::StatusInit() {
  ResetLeso();
  ResetRollLeso();
  ResetPitchLeso();
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
  normal_translation_wheel_sync_torque_ = 0.0f;
  normal_pivot_center_trim_torque_ = 0.0f;
  normal_pivot_center_prev_speed_ = 0.0f;
  normal_zero_speed_trim_torque_ = 0.0f;
  normal_zero_speed_prev_speed_ = 0.0f;
  normal_zero_speed_integral_delay_count_ = 0U;
  normal_zero_speed_correction_ = 0.0f;
  normal_lqr_wheel_common_torque_ = 0.0f;
  normal_speed_ref_ = 0.0f;
  normal_speed_ref_rate_ = 0.0f;
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
  normal_airborne_seen_ = false;
  normal_touchdown_active_ = false;
  normal_touchdown_left_contact_ = false;
  normal_touchdown_right_contact_ = false;
  normal_touchdown_left_contact_count_ = 0U;
  normal_touchdown_right_contact_count_ = 0U;
  normal_touchdown_timer_ = 0U;
  normal_touchdown_stable_count_ = 0U;
  normal_touchdown_ref_l_ = OFF_GROUND_LEG_LENGTH;
  normal_touchdown_ref_r_ = OFF_GROUND_LEG_LENGTH;
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
  jump_landing_pitch_ready_count_ = 0;
  jump_airborne_confirm_count_ = 0;
  jump_extend_ready_l_ = false;
  jump_extend_ready_r_ = false;
  jump_airborne_confirmed_ = false;
  jump_landing_pitch_ready_ = false;
  jump_normal_handoff_count_ = 0;
  normal_airborne_wheel_control_active_ = false;
  normal_airborne_wheel_ref_l_ = 0.0f;
  normal_airborne_wheel_ref_r_ = 0.0f;
  normal_airborne_wheel_integral_l_ = 0.0f;
  normal_airborne_wheel_integral_r_ = 0.0f;
  normal_airborne_pitch_ref_integral_ = 0.0f;
  normal_airborne_forward_speed_ref_ = 0.0f;
  normal_airborne_theta_ref_ = 0.0f;
  jump_wheel_integral_l_ = 0.0f;
  jump_wheel_integral_r_ = 0.0f;
  jump_wheel_pitch_ref_integral_ = 0.0f;
  jump_wheel_speed_ref_l_ = 0.0f;
  jump_wheel_speed_ref_r_ = 0.0f;
  jump_active_wheel_ref_l_ = 0.0f;
  jump_active_wheel_ref_r_ = 0.0f;
  jump_airborne_forward_speed_ref_ = 0.0f;
  jump_airborne_theta_ref_ = 0.0f;
  jump_landing_speed_scale_ = 1.0f;
  jump_land_phi0_ref_l_ = 0.0f;
  jump_land_phi0_ref_r_ = 0.0f;
  jump_landing_brake_ref_l_ = 0.0f;
  jump_landing_brake_ref_r_ = 0.0f;
  jump_land_deploy_ref_ = JUMP_RETRACT_LENGTH;
  jump_touchdown_ref_l_ = JUMP_LAND_PREP_LENGTH;
  jump_touchdown_ref_r_ = JUMP_LAND_PREP_LENGTH;
  jump_left_contact_count_ = jump_right_contact_count_ = 0U;
  jump_left_release_count_ = jump_right_release_count_ = 0U;
  jump_left_blocked_count_ = jump_right_blocked_count_ = 0U;
  dm_auto_enable_wait_count_ = 0U;
  dm_auto_enable_cooldown_count_ = 0U;
  dm_auto_enable_attempts_ = 0U;
  dm_auto_enable_pending_ = false;
  jump_left_contact_ = jump_right_contact_ = false;
  jump_left_blocked_contact_ = jump_right_blocked_contact_ = false;
  jump_both_contact_latched_ = false;
  jump_touchdown_capture_l_ = false;
  jump_touchdown_capture_r_ = false;
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
  jump_landing_pitch_ready_count_ = 0;
  jump_airborne_confirm_count_ = 0;
  jump_extend_ready_l_ = false;
  jump_extend_ready_r_ = false;
  jump_airborne_confirmed_ = false;
  jump_landing_pitch_ready_ = false;
  jump_wheel_integral_l_ = 0.0f;
  jump_wheel_integral_r_ = 0.0f;
  jump_wheel_pitch_ref_integral_ = 0.0f;
  jump_active_wheel_ref_l_ = 0.0f;
  jump_active_wheel_ref_r_ = 0.0f;
  jump_airborne_forward_speed_ref_ = 0.0f;
  jump_airborne_theta_ref_ = 0.0f;
  jump_landing_speed_scale_ = 1.0f;
  jump_land_phi0_ref_l_ = 0.0f;
  jump_land_phi0_ref_r_ = 0.0f;
  jump_landing_brake_ref_l_ = 0.0f;
  jump_landing_brake_ref_r_ = 0.0f;
  jump_land_deploy_ref_ = JUMP_RETRACT_LENGTH;
  jump_touchdown_ref_l_ = JUMP_LAND_PREP_LENGTH;
  jump_touchdown_ref_r_ = JUMP_LAND_PREP_LENGTH;
  jump_left_contact_count_ = jump_right_contact_count_ = 0U;
  jump_left_release_count_ = jump_right_release_count_ = 0U;
  jump_left_blocked_count_ = jump_right_blocked_count_ = 0U;
  jump_left_contact_ = jump_right_contact_ = false;
  jump_left_blocked_contact_ = jump_right_blocked_contact_ = false;
  jump_both_contact_latched_ = false;
  jump_touchdown_capture_l_ = false;
  jump_touchdown_capture_r_ = false;
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
  // The observer model is valid only for steady, grounded NORMAL operation.
  // Never carry an input-disturbance estimate across a hybrid-state change.
  ResetLeso();
  ResetRollLeso();
  ResetPitchLeso();
  // Terrain geometry belongs exclusively to grounded NORMAL. Clear its
  // filtered leg-length command on every state edge so it cannot enter
  // RECOVER, JUMP or the next NORMAL hand-off.
  roll_len_delta_cmd_ = 0.0f;
  normal_translation_wheel_sync_torque_ = 0.0f;
  const RobotStatus previous_state = robot_status;
  // Preserve the flight result before ResetJumpState() clears the jump-only
  // latches.  RETRACT may now hand an airborne robot directly to NORMAL, so
  // that transition must enter NORMAL's off-ground controller immediately
  // instead of being treated as a grounded post-landing hand-off.
  const bool previous_jump_airborne =
      previous_state == STATE_JUMPING &&
      (jump_airborne_confirmed_ || jump_liftoff_seen_ || GetOffGround());
  const float previous_left_wheel_speed = left_wheel.Get_Now_Omega();
  const float previous_right_wheel_speed = right_wheel.Get_Now_Omega();
  const float previous_jump_forward_speed = jump_airborne_forward_speed_ref_;
  const float previous_jump_theta_ref = jump_airborne_theta_ref_;
  // 当从跳跃状态切换到其他状态时，会先清除跳跃状态的子状态
  if (robot_status == STATE_JUMPING) {
    ResetJumpState();
  }
  // 当从上台阶状态切换到其他状态时，会先清除上台阶状态的子状态
  if (robot_status == STATE_STEP_UP) {
    ResetStepUpState();
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
      EnableAllJointMotors();
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
    // Clear stale diagnostics from a previous operator ESTOP. Any new ESTOP
    // source will write its own reason before entering STATE_ESTOP.
    estop_reason_ = 0U;
    jump_status = JUMP_COMPRESS;
    jump_timer = 0;
    jump_length_ready_count_ = 0;
    jump_retract_hold_count_ = 0;
    jump_landing_ready_count_ = 0;
    jump_landing_pitch_ready_count_ = 0;
    jump_airborne_confirm_count_ = 0;
    jump_extend_ready_l_ = false;
    jump_extend_ready_r_ = false;
    jump_airborne_confirmed_ = false;
    jump_landing_pitch_ready_ = false;
    jump_wheel_integral_l_ = 0.0f;
    jump_wheel_integral_r_ = 0.0f;
    jump_wheel_pitch_ref_integral_ = 0.0f;
    jump_wheel_speed_ref_l_ = 0.0f;
    jump_wheel_speed_ref_r_ = 0.0f;
    jump_active_wheel_ref_l_ = 0.0f;
    jump_active_wheel_ref_r_ = 0.0f;
    jump_airborne_forward_speed_ref_ = 0.0f;
    jump_airborne_theta_ref_ = 0.0f;
    jump_landing_speed_scale_ = 1.0f;
    jump_land_phi0_ref_l_ = 0.0f;
    jump_land_phi0_ref_r_ = 0.0f;
    jump_landing_brake_ref_l_ = 0.0f;
    jump_landing_brake_ref_r_ = 0.0f;
    jump_land_deploy_ref_ = JUMP_RETRACT_LENGTH;
    jump_touchdown_ref_l_ = JUMP_LAND_PREP_LENGTH;
    jump_touchdown_ref_r_ = JUMP_LAND_PREP_LENGTH;
    jump_left_contact_count_ = jump_right_contact_count_ = 0U;
    jump_left_release_count_ = jump_right_release_count_ = 0U;
    jump_left_blocked_count_ = jump_right_blocked_count_ = 0U;
    jump_left_contact_ = jump_right_contact_ = false;
    jump_left_blocked_contact_ = jump_right_blocked_contact_ = false;
    jump_both_contact_latched_ = false;
    jump_touchdown_capture_l_ = false;
    jump_touchdown_capture_r_ = false;
    jump_landing_zero_cross_l_ = false;
    jump_landing_zero_cross_r_ = false;
    jump_pitch_integral_ = 0.0f;
    jump_roll_integral_ = 0.0f;
    jump_liftoff_seen_ = false;
    jump_state_ = false;
  } else if (new_state == STATE_STEP_UP) {
    // 接近/撞击阶段已并入 NORMAL，撞击触发后进入固定动作（让空间→收腿→展腿）
    step_up_status = STEP_UP_CLEAR_LEG;
    step_up_timer = 0;
    step_up_impact_cnt_ = 0;
    step_up_state_ = false;        // 进入后清零
    step_up_wheel_stopped_ = false; // 重置轮速两个一次性触发锁存
    step_up_wheel_resumed_ = false;
    // 清除 NORMAL 控制残留
    normal_speed_ref_ = 0;
    normal_pivot_yaw_rate_ref_ = 0;
  }
  if (new_state == STATE_NORMAL &&
      (previous_state == STATE_RECOVERING ||
       previous_state == STATE_JUMPING ||
       previous_state == STATE_STEP_UP)) {
    estop_reason_ = 0U;
    normal_airborne_seen_ = false;
    normal_touchdown_active_ = false;
    normal_touchdown_left_contact_ = false;
    normal_touchdown_right_contact_ = false;
    normal_touchdown_left_contact_count_ = 0U;
    normal_touchdown_right_contact_count_ = 0U;
    normal_touchdown_timer_ = 0U;
    normal_touchdown_stable_count_ = 0U;
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
    normal_zero_speed_prev_speed_ = 0.0f;
    normal_zero_speed_integral_delay_count_ = 0U;
    normal_zero_speed_correction_ = 0.0f;
    normal_lqr_wheel_common_torque_ = 0.0f;
    normal_speed_ref_ = 0.0f;
    normal_speed_ref_rate_ = 0.0f;
    normal_wheel_center_speed_ = 0.0f;
    normal_wheel_diff_speed_ = 0.0f;
    normal_pivot_leg_sync_force_ = 0.0f;
    normal_pivot_leg_sync_integral_ = 0.0f;
    normal_pivot_left_speed_integral_ = 0.0f;
    normal_pivot_right_speed_integral_ = 0.0f;
    normal_pivot_pitch_integral_ = 0.0f;
    normal_pivot_pitch_target_ = 0.0f;
    normal_handoff_leg_grace_count_ = kNormalHandoffLegGraceTicks;
    normal_airborne_wheel_control_active_ = previous_jump_airborne;
    normal_airborne_wheel_ref_l_ = previous_left_wheel_speed;
    normal_airborne_wheel_ref_r_ = previous_right_wheel_speed;
    normal_airborne_wheel_integral_l_ = 0.0f;
    normal_airborne_wheel_integral_r_ = 0.0f;
    normal_airborne_pitch_ref_integral_ = 0.0f;
    normal_airborne_forward_speed_ref_ =
        previous_jump_airborne ? previous_jump_forward_speed : 0.0f;
    normal_airborne_theta_ref_ =
        previous_jump_airborne ? previous_jump_theta_ref : 0.0f;
    // The 250 ms grounded hand-off is only valid after a real landing.  When
    // RETRACT finishes in flight, keep the already-confirmed airborne state so
    // NORMAL immediately selects its off-ground gains and 0.40 m leg target.
    if (previous_jump_airborne) {
      off_ground_ = true;
      off_ground_enter_count_ = 0U;
      off_ground_exit_count_ = 0U;
      jump_normal_handoff_count_ = 0U;
    } else {
      jump_normal_handoff_count_ =
          (previous_state == STATE_JUMPING) ? JUMP_NORMAL_HANDOFF_TICKS : 0U;
      // 非空中交接（STEP_UP 完成 / 落地后的跳跃 / 起身）都处于着地状态，
      // 清掉过渡动作里可能被拉成 true 的离地标志，避免 NORMAL 用离地增益起步。
      off_ground_ = false;
      off_ground_enter_count_ = 0U;
      off_ground_exit_count_ = 0U;
    }
  } else if (new_state != STATE_NORMAL) {
    normal_airborne_seen_ = false;
    normal_touchdown_active_ = false;
    normal_touchdown_left_contact_ = false;
    normal_touchdown_right_contact_ = false;
    normal_touchdown_left_contact_count_ = 0U;
    normal_touchdown_right_contact_count_ = 0U;
    normal_touchdown_timer_ = 0U;
    normal_touchdown_stable_count_ = 0U;
    normal_handoff_active_ = false;
    heading_initialized_ = false;
    translation_command_active_ = false;
    yaw_command_active_ = false;
    normal_pivot_yaw_rate_ref_ = 0.0f;
    normal_pivot_yaw_torque_cmd_ = 0.0f;
    normal_pivot_center_trim_torque_ = 0.0f;
    normal_pivot_center_prev_speed_ = 0.0f;
    normal_zero_speed_trim_torque_ = 0.0f;
    normal_zero_speed_prev_speed_ = 0.0f;
    normal_zero_speed_integral_delay_count_ = 0U;
    normal_zero_speed_correction_ = 0.0f;
    normal_lqr_wheel_common_torque_ = 0.0f;
    normal_speed_ref_ = 0.0f;
    normal_speed_ref_rate_ = 0.0f;
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
    normal_airborne_wheel_control_active_ = false;
    normal_airborne_wheel_integral_l_ = 0.0f;
    normal_airborne_wheel_integral_r_ = 0.0f;
    normal_airborne_pitch_ref_integral_ = 0.0f;
    normal_airborne_forward_speed_ref_ = 0.0f;
    normal_airborne_theta_ref_ = 0.0f;
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

  // Re-enable all four DM joint drives independently of the chassis state.
  // This deliberately runs before every early return below, so NORMAL, JUMP,
  // RECOVER, JOINT_DEBUG and even an operator-requested ESTOP all execute the
  // same bounded recovery sequence.  ESTOP still commands zero torque.  A
  // real driver fault is never cleared or hidden.
  const uint8_t dm_fault_at_entry = GetDmFaultMaskNow();
  const uint8_t dm_reenable_at_entry =
      GetDmOfflineMaskNow() | GetDmDisabledMaskNow();
  if (dm_fault_at_entry == 0U && dm_reenable_at_entry != 0U) {
    if (dm_auto_enable_pending_) {
      if (dm_auto_enable_attempts_ < DM_AUTO_REENABLE_MAX_ATTEMPTS &&
          dm_auto_enable_wait_count_ > 0U &&
          (dm_auto_enable_wait_count_ % DM_AUTO_REENABLE_RETRY_TICKS) == 0U) {
        EnableAllJointMotors();
        dm_auto_enable_attempts_++;
      }
      if (dm_auto_enable_wait_count_ < DM_AUTO_REENABLE_WAIT_TICKS) {
        dm_auto_enable_wait_count_++;
      } else {
        // Stop masking the diagnostic after this bounded attempt.  In an
        // operational state the normal diagnostic path below enters ESTOP;
        // in ESTOP the motors remain zero-torque.  Retry later so a drive that
        // becomes receptive after the impact can still be re-enabled.
        dm_auto_enable_pending_ = false;
        dm_auto_enable_cooldown_count_ = DM_AUTO_REENABLE_COOLDOWN_TICKS;
      }
    } else if (dm_auto_enable_cooldown_count_ > 0U) {
      dm_auto_enable_cooldown_count_--;
    } else {
      EnableAllJointMotors();
      dm_auto_enable_pending_ = true;
      dm_auto_enable_wait_count_ = 0U;
      dm_auto_enable_attempts_ = 1U;
    }
  } else {
    dm_auto_enable_wait_count_ = 0U;
    dm_auto_enable_cooldown_count_ = 0U;
    dm_auto_enable_attempts_ = 0U;
    dm_auto_enable_pending_ = false;
  }

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
  // Diagnose and latch DM failures.  During the bounded, state-independent
  // Enable sequence above, keep the current controller alive long enough for
  // fresh feedback to arrive.  A driver fault is never masked.
  const bool waiting_for_manual_enable =
      robot_status == STATE_RECOVERING && recover_enable_pending_;
  const uint8_t dm_fault_now =
      waiting_for_manual_enable ? 0U : GetDmFaultMaskNow();
  const uint8_t dm_offline_raw =
      waiting_for_manual_enable ? 0U : GetDmOfflineMaskNow();
  const uint8_t dm_disabled_raw =
      waiting_for_manual_enable ? 0U : GetDmDisabledMaskNow();
  uint8_t dm_offline_now = dm_offline_raw;
  uint8_t dm_disabled_now = dm_disabled_raw;
  if (dm_fault_now == 0U && dm_auto_enable_pending_) {
    dm_offline_now = 0U;
    dm_disabled_now = 0U;
  }
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
      jump_airborne_forward_speed_ref_ =
          0.5f * k_wheel_radius *
          (right_wheel.Get_Now_Omega() - left_wheel.Get_Now_Omega());
      jump_airborne_theta_ref_ = 0.0f;
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      jump_status = JUMP_ASCEND;
      jump_timer = 0;
      jump_length_ready_count_ = 0;
      jump_retract_hold_count_ = 0;
      jump_extend_ready_l_ = false;
      jump_extend_ready_r_ = false;
      jump_airborne_confirm_count_ = 0;
      jump_airborne_confirmed_ = false;
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

  // 上台阶状态：检测是否完成（若未完成，则上台阶动作的保持）
  if (robot_status == STATE_STEP_UP) {
    if (IsStepUpComplete()) {
      // 清理子状态的步骤在 ChangeState 函数里，结束后直接接回普通状态
      ChangeState(STATE_NORMAL);
    }
    return;
  }

  // ====== 以下仅从 STATE_NORMAL 进入 ======
  if (robot_status != STATE_NORMAL) {
    return;
  }

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

  // 上台阶触发：使能开关(step_up_flag==2)打开后，撞击检测命中 → 断开 LQR 进入固定动作。
  // 接近/撞击阶段保持在 NORMAL 下由操作者遥控直行，撞击检测见 IsStepUpImpactDetected()。
  if (IsStepUpImpactDetected()) {
    ChangeState(STATE_STEP_UP);
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
  //上台阶
  if (robot_status == STATE_STEP_UP) {
    target_speed_ = 0.0f;
    target_w_rotation_ = 0.0f;
    target_dist_ = 0.0f;
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
  case STATE_STEP_UP: {
    StepUpCalc();
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
  const bool jump_landing_handoff = jump_normal_handoff_count_ > 0U;
  const bool normal_grounded = jump_landing_handoff || !GetOffGround();

  // NORMAL step-down landing detector.  Do not reuse this path for jump or
  // recovery: NormalCalc() is called only in STATE_NORMAL.  Once true flight
  // has been confirmed, latch each leg independently on its first sustained
  // load so an edge-first landing cannot switch the still-airborne leg to a
  // short ground target.
  if (GetOffGround() && !normal_airborne_seen_ &&
      !normal_touchdown_active_) {
    if (!normal_airborne_wheel_control_active_) {
      // Ordinary NORMAL ledge departure: capture the last ground-referenced
      // axle speed. A JUMP hand-off already carries the launch speed captured
      // on the 2 -> 1 edge and must not be overwritten by airborne wheel spin.
      normal_airborne_forward_speed_ref_ = normal_wheel_center_speed_;
      normal_airborne_theta_ref_ =
          0.5f * (left_leg_.GetTheta() + right_leg_.GetTheta());
    }
    normal_airborne_seen_ = true;
    normal_touchdown_left_contact_ = false;
    normal_touchdown_right_contact_ = false;
    normal_touchdown_left_contact_count_ = 0U;
    normal_touchdown_right_contact_count_ = 0U;
    normal_touchdown_timer_ = 0U;
    normal_touchdown_stable_count_ = 0U;
    normal_touchdown_ref_l_ = OFF_GROUND_LEG_LENGTH;
    normal_touchdown_ref_r_ = OFF_GROUND_LEG_LENGTH;
  }

  if (normal_airborne_seen_ || normal_touchdown_active_) {
    const bool left_contact_raw =
        left_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
    const bool right_contact_raw =
        right_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
    if (!normal_touchdown_left_contact_) {
      if (left_contact_raw) {
        if (normal_touchdown_left_contact_count_ <
            NORMAL_TOUCHDOWN_CONTACT_CONFIRM_TICKS) {
          normal_touchdown_left_contact_count_++;
        }
        if (normal_touchdown_left_contact_count_ >=
            NORMAL_TOUCHDOWN_CONTACT_CONFIRM_TICKS) {
          normal_touchdown_left_contact_ = true;
          normal_touchdown_ref_l_ = left_leg_.GetLegLen();
        }
      } else {
        normal_touchdown_left_contact_count_ = 0U;
      }
    }
    if (!normal_touchdown_right_contact_) {
      if (right_contact_raw) {
        if (normal_touchdown_right_contact_count_ <
            NORMAL_TOUCHDOWN_CONTACT_CONFIRM_TICKS) {
          normal_touchdown_right_contact_count_++;
        }
        if (normal_touchdown_right_contact_count_ >=
            NORMAL_TOUCHDOWN_CONTACT_CONFIRM_TICKS) {
          normal_touchdown_right_contact_ = true;
          normal_touchdown_ref_r_ = right_leg_.GetLegLen();
        }
      } else {
        normal_touchdown_right_contact_count_ = 0U;
      }
    }

    if (!normal_touchdown_active_ &&
        (normal_touchdown_left_contact_ ||
         normal_touchdown_right_contact_)) {
      normal_touchdown_active_ = true;
      normal_touchdown_timer_ = 0U;
      normal_touchdown_stable_count_ = 0U;
      left_leg_len_.Clear();
      right_leg_len_.Clear();
    }

    if (normal_touchdown_active_) {
      if (normal_touchdown_timer_ < NORMAL_TOUCHDOWN_MAX_TICKS)
        normal_touchdown_timer_++;
      const bool both_contact = normal_touchdown_left_contact_ &&
                                normal_touchdown_right_contact_;
      const bool touchdown_stable =
          both_contact &&
          left_leg_.GetLegLen() > NORMAL_TOUCHDOWN_MIN_SAFE_LENGTH &&
          right_leg_.GetLegLen() > NORMAL_TOUCHDOWN_MIN_SAFE_LENGTH &&
          fabsf(left_leg_.GetLegSpeed()) < NORMAL_TOUCHDOWN_LEG_SPEED_OK &&
          fabsf(right_leg_.GetLegSpeed()) < NORMAL_TOUCHDOWN_LEG_SPEED_OK &&
          fabsf(INS.Gyro[0]) < NORMAL_TOUCHDOWN_BODY_RATE_OK &&
          fabsf(INS.Gyro[1]) < NORMAL_TOUCHDOWN_BODY_RATE_OK;
      if (normal_touchdown_timer_ >= NORMAL_TOUCHDOWN_MIN_TICKS &&
          touchdown_stable) {
        if (normal_touchdown_stable_count_ <
            NORMAL_TOUCHDOWN_STABLE_TICKS) {
          normal_touchdown_stable_count_++;
        }
      } else {
        normal_touchdown_stable_count_ = 0U;
      }

      const bool touchdown_complete =
          normal_touchdown_stable_count_ >= NORMAL_TOUCHDOWN_STABLE_TICKS;
      const bool touchdown_timeout =
          both_contact &&
          normal_touchdown_timer_ >= NORMAL_TOUCHDOWN_MAX_TICKS;
      if (touchdown_complete || touchdown_timeout) {
        // Resume the ordinary NORMAL controller through its existing measured
        // length hand-off ramp; never step directly back to the operator's
        // potentially short target.
        normal_touchdown_active_ = false;
        normal_airborne_seen_ = false;
        normal_handoff_active_ = true;
        normal_handoff_len_l_ = left_leg_.GetLegLen();
        normal_handoff_len_r_ = right_leg_.GetLegLen();
        left_leg_len_.Clear();
        right_leg_len_.Clear();
      }
    }
  }

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
  const bool in_place_yaw = translation_idle && yaw_turn;
  // Legacy pivot-only branches below are compile-time disabled.  They remain
  // temporarily in the source for traceability, but can no longer select a
  // different NORMAL balance law.
  constexpr bool pivot_turn = false;
  // LQRCalc() applies the same fixed NORMAL Pitch reference for idle,
  // translation and yaw. Reuse it in the parallel common-hip Pitch path.
  const float normal_pitch_target = normal_pivot_pitch_target_;
  const float normal_pitch = INS.Pitch - normal_pitch_target;

  // NORMAL gets its own second-stage command shaper.  The SBUS ramp is shared
  // by every state, so changing it would also change recovery behaviour.  A
  // bounded speed slope here reduces the pitch excursion caused by a sudden
  // translation command.  A pivot always requests exactly zero common speed.
  const float normal_speed_goal = translation_idle ? 0.0f : target_speed_;
  const float speed_ref_dt =
      (controller_dt_ > 0.0f && controller_dt_ < 0.02f) ? controller_dt_
                                                        : 0.001f;
  const float speed_ref_error = normal_speed_goal - normal_speed_ref_;
  const float speed_ref_omega = kNormalSpeedRefNaturalFrequency;
  const float desired_jerk =
      speed_ref_omega * speed_ref_omega * speed_ref_error -
      2.0f * kNormalSpeedRefDampingRatio * speed_ref_omega *
          normal_speed_ref_rate_;
  const float limited_jerk =
      ClampAbs(desired_jerk, kNormalSpeedRefJerkLimit);
  normal_speed_ref_rate_ += limited_jerk * speed_ref_dt;

  // Preserve the established acceleration/deceleration ceilings. Increasing
  // the magnitude of a same-sign request uses the acceleration limit; stopping
  // or reversing uses the larger deceleration limit.
  const bool increasing_speed_magnitude =
      normal_speed_goal * normal_speed_ref_ >= 0.0f &&
      fabsf(normal_speed_goal) > fabsf(normal_speed_ref_);
  const float speed_ref_rate_limit = increasing_speed_magnitude
                                         ? kNormalSpeedRefAccel
                                         : kNormalSpeedRefDecel;
  normal_speed_ref_rate_ =
      ClampAbs(normal_speed_ref_rate_, speed_ref_rate_limit);
  const float previous_speed_ref_error = speed_ref_error;
  normal_speed_ref_ += normal_speed_ref_rate_ * speed_ref_dt;

  // Snap only at the final crossing to avoid a residual reference-rate tail;
  // the path up to this point remains continuous and jerk limited.
  const float next_speed_ref_error = normal_speed_goal - normal_speed_ref_;
  if (previous_speed_ref_error * next_speed_ref_error <= 0.0f &&
      fabsf(previous_speed_ref_error) > 0.0f) {
    normal_speed_ref_ = normal_speed_goal;
    normal_speed_ref_rate_ = 0.0f;
  } else if (fabsf(next_speed_ref_error) < 0.001f &&
             fabsf(normal_speed_ref_rate_) < 0.01f) {
    normal_speed_ref_ = normal_speed_goal;
    normal_speed_ref_rate_ = 0.0f;
  }
  // Smoothly hand the common-wheel channel from the moving LQR target to the
  // exact-zero midpoint loop. This removes the old torque edge at 0.03 m/s.
  float normal_zero_speed_blend = 0.0f;
  if (translation_idle) {
    const float zero_ref_blend = ClampRange(
        (kNormalZeroSpeedBlendStartRef - fabsf(normal_speed_ref_)) /
            (kNormalZeroSpeedBlendStartRef - kNormalZeroSpeedBlendFullRef),
        0.0f, 1.0f);
    // The axle-midpoint loop must become stronger, not disappear, when the
    // measured centre speed drifts during a zero-vx yaw command.  Gate only on
    // the still-decaying translation reference; once it is zero, retain full
    // midpoint authority for any measured drift magnitude.
    normal_zero_speed_blend = zero_ref_blend;
  }

  // Let the historical positive drivetrain bias follow the already-slewed
  // speed reference all the way to zero.  Basing this on the raw stick idle
  // flag made the bias disappear in one control tick while normal_speed_ref_
  // was still decelerating, which kicked the LQR hip rows during a forward
  // stop.  A pivot still forces both the reference and bias to exactly zero.
  const float normal_bias_scale =
      ClampRange(fabsf(normal_speed_ref_) / kNormalSpeedBiasRampSpeed, 0.0f,
                 1.0f);
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
  const float center_speed_lpf_alpha = kNormalWheelSpeedLpfAlpha;
  normal_wheel_center_speed_ +=
      center_speed_lpf_alpha *
      (encoder_center_speed - normal_wheel_center_speed_);
  normal_wheel_diff_speed_ += kNormalWheelSpeedLpfAlpha *
                              (encoder_diff_speed - normal_wheel_diff_speed_);
  // Keep the same wheel-only yaw controller alive after the reference reaches
  // zero until measured yaw motion has settled.  This is continuous zero-rate
  // feedback, not an exit state.
  const bool yaw_control_active =
      yaw_turn || fabsf(INS.YawSpeed) > kPivotExitYawRateSettled ||
      (translation_idle &&
       fabsf(normal_wheel_diff_speed_) > kPivotExitDiffSpeedSettled);

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
      if (!translation_idle) {
        inertial_ff = mass_eff * (leg_len_mean / (2.0f * LEG_FF_WHEEL_TRACK)) *
                      vel_ * INS.Gyro[2];
      }
    }

    // NORMAL-only low-bandwidth roll controller.  Do not numerically
    // differentiate the Euler angle at 1 kHz: the resulting D term was visible
    // on I10 as roughly -20..50 N noise.  The IMU gyro is the measured roll
    // rate, so use it directly for damping and filter the final axial-force
    // difference before it reaches VMC.
    const float roll_target = kNormalRollZeroOffset;
    const float roll_error = roll_target - INS.Roll;

    // A differential support force controls fast roll motion, but it cannot
    // remove the geometric constraint imposed by equal stiff leg-length
    // Reconstruct the contact-height difference directly from body Roll and
    // the measured vertical components of both legs. This works on arbitrary
    // uneven terrain and, unlike the former PI/terrain-confirm path, carries no
    // terrain memory onto flat ground. kRollLenDirection is the single sign to
    // flip if a static supported test shows the opposite convention.
    const bool roll_length_safe =
        normal_grounded && !normal_airborne_seen_ && !normal_touchdown_active_ &&
        !normal_handoff_active_ && !jump_landing_handoff && !in_place_yaw &&
        controller_dt_ > 0.0f && controller_dt_ < 0.02f &&
        GetDmFaultMaskNow() == 0U && GetDmOfflineMaskNow() == 0U &&
        GetDmDisabledMaskNow() == 0U;
    float roll_len_target = 0.0f;
    // During an in-place turn the differential wheel reaction creates a
    // transient Roll/leg-length difference even on flat ground. Do not feed
    // that motion back as an estimated terrain step: command equal leg lengths
    // and leave Roll rejection to the differential force loop plus the
    // yaw-specific leg synchronizer below.
    if (roll_length_safe) {
      const float left_vertical_length =
          left_leg_.GetLegLen() * arm_cos_f32(left_leg_.GetTheta());
      const float right_vertical_length =
          right_leg_.GetLegLen() * arm_cos_f32(right_leg_.GetTheta());
      float estimated_ground_height_diff =
          LEG_FF_WHEEL_TRACK * arm_sin_f32(INS.Roll - roll_target) -
          (left_vertical_length - right_vertical_length);
      if (fabsf(estimated_ground_height_diff) <
          kRollTerrainHeightDeadband) {
        estimated_ground_height_diff = 0.0f;
      }
      // Positive ground-height difference means the left wheel is higher, so
      // the left leg must be the short leg. roll_len_delta_cmd_ is one half of
      // the final left/right reference difference.
      roll_len_target = ClampAbs(
          0.5f * kRollLenDirection * estimated_ground_height_diff,
          kRollLenMax);
    }
    const float roll_len_dt =
        (controller_dt_ > 0.0f && controller_dt_ < 0.02f) ? controller_dt_
                                                          : 0.001f;
    const float roll_len_alpha =
        roll_len_dt / (kRollLenTimeConstant + roll_len_dt);
    // Do not chase millimetre-scale geometry noise once the terrain height is
    // steady. Apply the dead zone to the target discrepancy rather than to the
    // absolute terrain height, so a large terrain transition retains the fast
    // filter/slew response and smoothly comes to rest without relay chatter.
    const float roll_len_target_error =
        roll_len_target - roll_len_delta_cmd_;
    const float roll_len_command_deadband =
        roll_length_safe ? kRollLenCommandDeadband : 0.0f;
    float roll_len_driven_error = 0.0f;
    if (roll_len_target_error > roll_len_command_deadband) {
      roll_len_driven_error =
          roll_len_target_error - roll_len_command_deadband;
    } else if (roll_len_target_error < -roll_len_command_deadband) {
      roll_len_driven_error =
          roll_len_target_error + roll_len_command_deadband;
    }
    const float roll_len_filtered =
        roll_len_delta_cmd_ + roll_len_alpha * roll_len_driven_error;
    const float previous_roll_len_delta = roll_len_delta_cmd_;
    roll_len_delta_cmd_ =
        SlewTowards(roll_len_delta_cmd_, roll_len_filtered,
                    kRollLenSlewPerSecond * roll_len_dt);

    // Apply the required difference by shortening only the low-side leg.  The
    // long leg remains exactly at the operator/common length reference instead
    // of being extended above it to preserve the former mean-length command.
    // roll_len_delta_cmd_ was historically a half-difference, so shorten the
    // opposite leg by twice its magnitude to retain the same differential
    // authority.  If the short leg reaches MIN_LEG_LENGTH, accept the reduced
    // Roll range rather than raising the anchored long leg.
    const float roll_base_left_ref = left_ref;
    const float roll_base_right_ref = right_ref;
    float roll_left_ref = roll_base_left_ref;
    float roll_right_ref = roll_base_right_ref;
    const float previous_left_offset_raw =
        previous_roll_len_delta > 0.0f ? -2.0f * previous_roll_len_delta
                                       : 0.0f;
    const float previous_right_offset_raw =
        previous_roll_len_delta < 0.0f ? 2.0f * previous_roll_len_delta
                                       : 0.0f;
    const float previous_left_terrain_offset =
        ClampRange(roll_base_left_ref + previous_left_offset_raw,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH) -
        roll_base_left_ref;
    const float previous_right_terrain_offset =
        ClampRange(roll_base_right_ref + previous_right_offset_raw,
                   MIN_LEG_LENGTH, MAX_LEG_LENGTH) -
        roll_base_right_ref;
    float left_terrain_offset = 0.0f;
    float right_terrain_offset = 0.0f;
    if (roll_len_delta_cmd_ > 0.0f) {
      left_terrain_offset = -2.0f * roll_len_delta_cmd_;
      roll_left_ref = left_ref + left_terrain_offset;
    } else if (roll_len_delta_cmd_ < 0.0f) {
      right_terrain_offset = 2.0f * roll_len_delta_cmd_;
      roll_right_ref = right_ref + right_terrain_offset;
    }
    left_ref = ClampRange(roll_left_ref, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    right_ref = ClampRange(roll_right_ref, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    left_terrain_offset = left_ref - roll_base_left_ref;
    right_terrain_offset = right_ref - roll_base_right_ref;

    // The shared length PID differentiates the measured length for damping.
    // Without matching reference-velocity feed-forward, that damping resists a
    // fast terrain command and forces the physical leg to lag its reference.
    // Add only the velocity of the differential terrain offset here; operator
    // length changes and every non-NORMAL state keep their established tuning.
    const float left_terrain_ref_speed =
        (left_terrain_offset - previous_left_terrain_offset) / roll_len_dt;
    const float right_terrain_ref_speed =
        (right_terrain_offset - previous_right_terrain_offset) / roll_len_dt;
    const float left_terrain_velocity_ff =
        ClampAbs(kRollLenVelocityFfGain * left_terrain_ref_speed,
                 kRollLenVelocityFfMax);
    const float right_terrain_velocity_ff =
        ClampAbs(kRollLenVelocityFfGain * right_terrain_ref_speed,
                 kRollLenVelocityFfMax);

    const float target_leg_diff = left_ref - right_ref;
    const float measured_leg_diff =
        left_leg_.GetLegLen() - right_leg_.GetLegLen();
    const float leg_diff_tracking_error =
        measured_leg_diff - target_leg_diff;

    const float roll_kp = kNormalRollKp;
    const float roll_kd = kNormalRollKd;
    const float roll_force_limit = kNormalRollOutMax;
    const float pivot_roll_leg_scale = 1.0f;
    // The LESO owns the low-frequency matched disturbance channel. Keeping the
    // old integrator in parallel would make the two estimators fight and would
    // retain force after a terrain/load transition.
    normal_pivot_roll_integral_ = 0.0f;
    roll_pd_force_ = 0.0f;
    if (fabsf(roll_error) > kRollDeadBand ||
        fabsf(INS.Gyro[0]) > kRollRateActivityThreshold) {
      roll_pd_force_ = pivot_roll_leg_scale *
                       ClampAbs(roll_kp * roll_error - roll_kd * INS.Gyro[0],
                                 roll_force_limit);
    }

    const bool roll_leso_enable =
#if ROLL_LESO_COMPENSATION_ENABLE
        normal_grounded && !normal_airborne_seen_ &&
        !normal_touchdown_active_ && !normal_handoff_active_ &&
        !jump_landing_handoff && pivot_roll_leg_scale > 0.0f &&
        controller_dt_ > 0.0f && controller_dt_ < 0.02f &&
        GetDmFaultMaskNow() == 0U && GetDmOfflineMaskNow() == 0U &&
        GetDmDisabledMaskNow() == 0U;
#else
        false;
#endif
    const float roll_leso_correction =
        UpdateRollLeso(roll_leso_enable, INS.Roll, INS.Gyro[0]);
    const float normal_roll_raw = ClampAbs(
        roll_pd_force_ + pivot_roll_leg_scale * roll_leso_correction,
        roll_force_limit);
    roll_force_cmd_ = ClampAbs(roll_force_cmd_, roll_force_limit);
    const float roll_lpf_alpha = kNormalRollForceLpfAlpha;
    roll_force_cmd_ += roll_lpf_alpha * (normal_roll_raw - roll_force_cmd_);
    const float roll_out = roll_force_cmd_;

    // Use one physical roll direction throughout NORMAL. Hardware terrain data
    // showed that the old non-pivot branch used the opposite sign and therefore
    // enlarged roll instead of levelling the body. Keep the independently
    // derived lateral-inertia feedforward direction unchanged.
    left_ff = gravity_ff_left + roll_out - inertial_ff +
              left_terrain_velocity_ff;
    right_ff = gravity_ff_right - roll_out + inertial_ff +
               right_terrain_velocity_ff;

  } else {
    roll_force_cmd_ = 0.0f;
    normal_pivot_roll_integral_ = 0.0f;
    roll_len_delta_cmd_ = 0.0f;
    roll_pd_force_ = 0.0f;
    ResetRollLeso();
    left_ref = OFF_GROUND_LEG_LENGTH;
    right_ref = OFF_GROUND_LEG_LENGTH;
  }

  if (normal_touchdown_active_) {
    // A contacted leg holds its measured touchdown length so compression
    // creates an outward spring force.  A leg that has not touched yet stays
    // extended and is not pulled toward the shorter operator target.
    left_ref = normal_touchdown_left_contact_
                   ? normal_touchdown_ref_l_
                   : OFF_GROUND_LEG_LENGTH;
    right_ref = normal_touchdown_right_contact_
                    ? normal_touchdown_ref_r_
                    : OFF_GROUND_LEG_LENGTH;
  }

  // NORMAL-only axle-midpoint speed loop. It remains active during a pivot:
  // common wheel speed must stay at zero while the differential channel turns
  // the chassis. Otherwise a drivetrain/pitch bias is added to both wheels and
  // moves the instantaneous rotation centre outside the robot.
  const bool zero_speed_integral_candidate =
      translation_idle && normal_grounded &&
      !normal_touchdown_active_ && !normal_handoff_active_ &&
      fabsf(normal_speed_ref_) <= kNormalZeroSpeedIntegralEnableRef;
  if (zero_speed_integral_candidate && controller_dt_ > 0.0f &&
      controller_dt_ < 0.02f) {
    if (normal_zero_speed_integral_delay_count_ <
        kNormalZeroSpeedIntegralDelayTicks) {
      normal_zero_speed_integral_delay_count_++;
    }
    if (normal_zero_speed_integral_delay_count_ >=
        kNormalZeroSpeedIntegralDelayTicks) {
      const float zero_cross_threshold_sq =
          kNormalZeroSpeedIntegralDeadband *
          kNormalZeroSpeedIntegralDeadband;
      if (normal_zero_speed_prev_speed_ * normal_wheel_center_speed_ <
          -zero_cross_threshold_sq) {
        // A repeated speed sign change indicates a balance oscillation, not a
        // constant drivetrain bias. Discard most stored trim before integrating
        // the new half-cycle.
        normal_zero_speed_trim_torque_ *=
            kNormalZeroSpeedZeroCrossRetention;
      }
      if (fabsf(normal_wheel_center_speed_) >
          kNormalZeroSpeedIntegralDeadband) {
        normal_zero_speed_trim_torque_ = ClampAbs(
            normal_zero_speed_trim_torque_ -
                kNormalZeroSpeedKi * normal_wheel_center_speed_ *
                    controller_dt_,
            kNormalZeroSpeedIntegralMax);
      }
    }
    normal_zero_speed_prev_speed_ = normal_wheel_center_speed_;
  } else {
    // A drive command, pivot, landing transition or invalid period immediately
    // discards idle-only stored energy.
    normal_zero_speed_trim_torque_ = 0.0f;
    normal_zero_speed_prev_speed_ = 0.0f;
    normal_zero_speed_integral_delay_count_ = 0U;
  }

  // Cancel only the slow, persistent midpoint drift during an in-place turn.
  // NORMAL LQR retains its normal speed and pitch states; the bounded P term
  // below stops fast common-speed growth, while this integral removes the
  // remaining bias that leaves one wheel nearly stationary.  It is discarded
  // immediately outside a pivot.
  normal_pivot_center_trim_torque_ = 0.0f;
  normal_pivot_center_prev_speed_ = 0.0f;

  // Normal mode attitude/common-wheel controller.
  LQRCalc();
  SynthesizeMotion();
  normal_lqr_wheel_common_torque_ = 0.5f * (l_wheel_T_ + r_wheel_T_);
  normal_zero_speed_correction_ = 0.0f;

  if (normal_grounded && controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
    // Pitch is already a state of the full LQR and has a direct common-hip P/D
    // path. A second integral on either common actuator merely moves the other
    // LQR states until the final common torque returns to zero.
    normal_pivot_pitch_integral_ = 0.0f;
    const float normal_pitch_hip_torque = ClampAbs(
        kNormalPivotHipPitchKp * normal_pitch +
            kNormalPivotHipPitchKd * INS.Gyro[1],
        kNormalPivotHipPitchTorqueMax);
    left_leg_T_ = ClampAbs(left_leg_T_ + normal_pitch_hip_torque, 40.0f);
    right_leg_T_ = ClampAbs(right_leg_T_ + normal_pitch_hip_torque, 40.0f);
  } else {
    normal_pivot_pitch_integral_ = 0.0f;
  }

  // Rebuild the grounded hip differential from one explicit synchronization
  // mode.  A pure common projection removed the damping needed when recovery
  // hands NORMAL two legs with unequal angles or opposite residual velocities;
  // they then continued travelling apart.  This bounded PD keeps the LQR's
  // common Pitch request, but permits only the differential torque that moves
  // the two measured leg angles toward each other.  Yaw remains wheel-only and
  // Roll remains in the differential axial-force path.
  if (normal_grounded) {
    const float leg_torque_common = 0.5f * (left_leg_T_ + right_leg_T_);
    const float leg_angle_diff =
        left_leg_.GetTheta() - right_leg_.GetTheta();
    const float leg_rate_diff =
        left_leg_.GetDotTheta() - right_leg_.GetDotTheta();
    const float leg_sync_torque = ClampAbs(
        kNormalHipSyncKp * leg_angle_diff +
            kNormalHipSyncKd * leg_rate_diff,
        kNormalHipSyncTorqueMax);
    left_leg_T_ = ClampAbs(leg_torque_common - leg_sync_torque, 40.0f);
    right_leg_T_ = ClampAbs(leg_torque_common + leg_sync_torque, 40.0f);
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
    const bool translation_wheel_sync_enable =
        normal_grounded && !translation_idle && !yaw_control_active &&
        !normal_airborne_seen_ && !normal_touchdown_active_ &&
        !normal_handoff_active_ && !jump_landing_handoff &&
        controller_dt_ > 0.0f && controller_dt_ < 0.02f &&
        GetDmFaultMaskNow() == 0U && GetDmOfflineMaskNow() == 0U &&
        GetDmDisabledMaskNow() == 0U;

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
    const float center_loop_min_scale = kNormalCenterLoopMinScale;
    center_loop_scale =
        ClampRange(center_loop_scale, center_loop_min_scale, 1.0f);
    // The pivot branch above owns its own midpoint loop. This branch fades in
    // only the straight-line idle correction as the translation ramp settles.
    if (normal_zero_speed_blend > 0.0f) {
      // A centred translation stick means an exact zero axle-midpoint target.
      const float center_speed_torque =
          normal_zero_speed_blend * center_loop_scale *
          ClampAbs(kNormalCenterSpeedTorqueSign * kNormalZeroSpeedKp *
                           (0.0f - normal_wheel_center_speed_) +
                       normal_zero_speed_trim_torque_,
                   kNormalZeroSpeedTorqueMax);
      normal_zero_speed_correction_ = center_speed_torque;
      wheel_common += center_speed_torque;
    }

    if (yaw_control_active) {
      // One wheel-only yaw-rate controller is used for all NORMAL commands.
      // The kinematic wheel-difference target is corrected by measured IMU yaw
      // rate.  When the command has slewed to zero this same expression changes
      // sign and brakes residual rotation without a controller hand-off.
      const float geometric_diff_speed =
          0.5f * LEG_FF_WHEEL_TRACK * target_w_rotation_;
      const float target_diff_speed = ClampAbs(
          geometric_diff_speed +
              kPivotYawRateToDiffSpeedKp *
                  (target_w_rotation_ - INS.YawSpeed),
          0.5f * LEG_FF_WHEEL_TRACK * YAW_SPEED_MAX);
      const float yaw_common_speed_target = normal_common_speed_target;
      const float target_left_speed =
          yaw_common_speed_target - target_diff_speed;
      const float target_right_speed =
          yaw_common_speed_target + target_diff_speed;
      normal_target_left_wheel_speed_ = target_left_speed;
      normal_target_right_wheel_speed_ = target_right_speed;
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
      const float yaw_balance_scale = pitch_yaw_scale * leg_yaw_scale;

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
      // With zero yaw rate and no residual rotation, use a small P-only
      // differential wheel-speed
      // loop while translating.  If the left wheel is slower, left-right speed
      // is negative, so wheel_diff becomes negative: left=common-diff gains
      // torque and right=common+diff loses torque.  The same relation also works
      // in reverse.  This is generic load sharing, not a bridge/stair detector.
      normal_pivot_yaw_torque_cmd_ = 0.0f;
      normal_pivot_left_speed_integral_ = 0.0f;
      normal_pivot_right_speed_integral_ = 0.0f;
      if (translation_wheel_sync_enable) {
        const float actual_left_speed =
            normal_wheel_center_speed_ - normal_wheel_diff_speed_;
        const float actual_right_speed =
            normal_wheel_center_speed_ + normal_wheel_diff_speed_;
        const float wheel_sync_raw = ClampAbs(
            kNormalTranslationWheelSyncKp *
                (actual_left_speed - actual_right_speed),
            kNormalTranslationWheelSyncMax);
        normal_translation_wheel_sync_torque_ +=
            kNormalTranslationWheelSyncLpfAlpha *
            (wheel_sync_raw - normal_translation_wheel_sync_torque_);
        wheel_diff = normal_translation_wheel_sync_torque_;
      } else {
        // Zero input, yaw, flight and hybrid contact transitions must not retain
        // a differential torque tail in the next control mode.
        normal_translation_wheel_sync_torque_ = 0.0f;
        wheel_diff = 0.0f;
      }
    }

    // Preserve the common/differential decomposition at saturation.  NORMAL
    // balance torque has first priority; yaw uses only the remaining symmetric
    // wheel authority and therefore cannot starve Pitch stabilization.
    if (yaw_control_active) {
      wheel_common = ClampAbs(wheel_common, kNormalWheelTorqueLimit);
      const float yaw_diff_limit =
          kNormalWheelTorqueLimit - fabsf(wheel_common);
      wheel_diff = ClampAbs(wheel_diff, yaw_diff_limit);
      l_wheel_T_ = wheel_common - wheel_diff;
      r_wheel_T_ = wheel_common + wheel_diff;
    } else if (translation_wheel_sync_enable) {
      // During straight-line load sharing, clamp the two physical wheel
      // requests independently.  This keeps the slower/blocked wheel at its
      // available limit while reducing the faster wheel.  A common-priority
      // allocator would erase all differential authority whenever the common
      // LQR request reached the 10 N.m limit.
      l_wheel_T_ =
          ClampAbs(wheel_common - wheel_diff, kNormalWheelTorqueLimit);
      r_wheel_T_ =
          ClampAbs(wheel_common + wheel_diff, kNormalWheelTorqueLimit);
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

  if (!normal_grounded || normal_airborne_seen_) {
    // The ordinary NORMAL length PID is deliberately filtered for quiet
    // ground driving.  Immediately after an airborne JUMP -> NORMAL hand-off
    // that filter deploys the legs too slowly and leaves too little stroke for
    // impact absorption.  Use a bounded, unfiltered airborne PD until the
    // touchdown detector captures either leg; the contacted side is then
    // replaced by the compliant impedance below in the same control cycle.
    if (!normal_touchdown_left_contact_) {
      left_leg_F_ = ClampRange(
          NORMAL_OFF_GROUND_EXTEND_KP *
                  (OFF_GROUND_LEG_LENGTH - left_leg_.GetLegLen()) -
              NORMAL_OFF_GROUND_EXTEND_KD * left_leg_.GetLegSpeed(),
          NORMAL_OFF_GROUND_FORCE_MIN, NORMAL_OFF_GROUND_FORCE_MAX);
    }
    if (!normal_touchdown_right_contact_) {
      right_leg_F_ = ClampRange(
          NORMAL_OFF_GROUND_EXTEND_KP *
                  (OFF_GROUND_LEG_LENGTH - right_leg_.GetLegLen()) -
              NORMAL_OFF_GROUND_EXTEND_KD * right_leg_.GetLegSpeed(),
          NORMAL_OFF_GROUND_FORCE_MIN, NORMAL_OFF_GROUND_FORCE_MAX);
    }
  }
  if (normal_touchdown_active_) {
    // Landing-only compliant impedance.  Near the mechanical minimum, add a
    // progressive virtual bump stop while the leg is still compressing.
    const float left_speed = left_leg_.GetLegSpeed();
    const float right_speed = right_leg_.GetLegSpeed();
    const float left_touchdown_force = CalculateLandingImpedanceForce(
        left_ref, left_leg_.GetLegLen(), left_speed,
        k_gravity_comp * arm_cos_f32(left_leg_.GetTheta()),
        NORMAL_TOUCHDOWN_KP, NORMAL_TOUCHDOWN_KD,
        NORMAL_TOUCHDOWN_FORCE_MIN, NORMAL_TOUCHDOWN_FORCE_MAX,
        NORMAL_TOUCHDOWN_SOFT_LIMIT, NORMAL_TOUCHDOWN_SOFT_STOP_KP,
        NORMAL_TOUCHDOWN_SOFT_STOP_KD, normal_touchdown_left_contact_);
    const float right_touchdown_force = CalculateLandingImpedanceForce(
        right_ref, right_leg_.GetLegLen(), right_speed,
        k_gravity_comp * arm_cos_f32(right_leg_.GetTheta()),
        NORMAL_TOUCHDOWN_KP, NORMAL_TOUCHDOWN_KD,
        NORMAL_TOUCHDOWN_FORCE_MIN, NORMAL_TOUCHDOWN_FORCE_MAX,
        NORMAL_TOUCHDOWN_SOFT_LIMIT, NORMAL_TOUCHDOWN_SOFT_STOP_KP,
        NORMAL_TOUCHDOWN_SOFT_STOP_KD, normal_touchdown_right_contact_);
    // Do not apply ground-impact impedance to a leg that is still airborne.
    // Its existing 0.40 m reference remains under the regular filtered PID.
    if (normal_touchdown_left_contact_) {
      left_leg_F_ = left_touchdown_force;
    }
    if (normal_touchdown_right_contact_) {
      right_leg_F_ = right_touchdown_force;
    }
  }

  // Track the commanded leg-length difference throughout NORMAL.  On flat
  // ground this target is zero; on a cross-slope it is the intentional geometric
  // roll correction above.  Driving the measured difference to zero here would
  // directly cancel that correction and force the body to follow the slope.
  if (normal_grounded && !normal_touchdown_active_) {
    // Differential wheel reaction compresses opposite legs for opposite yaw
    // directions.  Use a firmer, still P/D-only synchronizer while yawing so
    // this reversible load does not become a visible leg-length split.
    const float leg_sync_kp =
        in_place_yaw ? kNormalYawLegSyncKp : kNormalLegSyncKp;
    const float leg_sync_kd =
        in_place_yaw ? kNormalYawLegSyncKd : kNormalLegSyncKd;
    const float leg_sync_force_max = kNormalLegSyncForceMax;
    const float measured_leg_length_diff =
        left_leg_.GetLegLen() - right_leg_.GetLegLen();
    const float target_leg_length_diff = left_ref - right_ref;
    const float leg_length_diff_error =
        measured_leg_length_diff - target_leg_length_diff;
    normal_pivot_leg_sync_integral_ = 0.0f;
    const float leg_sync_raw = ClampAbs(
        leg_sync_kp * leg_length_diff_error +
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
  if (roll_leso_active_) {
    // Feed the observer the final virtual axial-force half-difference after
    // gravity feedforward, length PID and leg synchronisation. This is the
    // differential input actually handed to VMC, not merely the Roll branch's
    // requested portion of it.
    roll_leso_applied_force_ = 0.5f * (left_leg_F_ - right_leg_F_);
  }
  if (GetOffGround()) {
    const float airborne_theta_goal = ClampAbs(
        AIRBORNE_LEG_FORWARD_SIGN * AIRBORNE_LEG_SPEED_ANGLE_GAIN *
            normal_airborne_forward_speed_ref_,
        AIRBORNE_LEG_ANGLE_MAX);
    normal_airborne_theta_ref_ = SlewTowards(
        normal_airborne_theta_ref_, airborne_theta_goal,
        AIRBORNE_LEG_ANGLE_SLEW_PER_TICK);
    // Replace the zero-theta off-ground LQR hip rows with an explicit
    // world-frame foot-placement PD. Wheel pitch control remains independent.
    left_leg_T_ = ClampAbs(
        AIRBORNE_LEG_ANGLE_KP *
                (normal_airborne_theta_ref_ - left_leg_.GetTheta()) -
            AIRBORNE_LEG_ANGLE_KD * left_leg_.GetDotTheta(),
        AIRBORNE_LEG_ANGLE_TORQUE_MAX);
    right_leg_T_ = ClampAbs(
        AIRBORNE_LEG_ANGLE_KP *
                (normal_airborne_theta_ref_ - right_leg_.GetTheta()) -
            AIRBORNE_LEG_ANGLE_KD * right_leg_.GetDotTheta(),
        AIRBORNE_LEG_ANGLE_TORQUE_MAX);
  } else {
    normal_airborne_theta_ref_ = 0.0f;
  }
  if (normal_airborne_wheel_control_active_ && GetOffGround()) {
    // NORMAL's off-ground gain matrix deliberately zeros both wheel rows.  A
    // direct airborne hand-off would therefore discard the reaction-wheel
    // pitch authority that was active in RETRACT. Continue the same verified
    // shaft-speed/pitch convention until the touchdown detector takes over.
    float airborne_dt = controller_dt_;
    if (airborne_dt <= 0.0f || airborne_dt > 0.02f)
      airborne_dt = 0.001f;

    normal_airborne_pitch_ref_integral_ = ClampAbs(
        normal_airborne_pitch_ref_integral_ -
            JUMP_RETRACT_WHEEL_PITCH_REF_KI * INS.Pitch * airborne_dt,
        JUMP_RETRACT_WHEEL_PITCH_REF_I_MAX);
    const float pitch_speed_ref_trim = ClampAbs(
        -JUMP_RETRACT_WHEEL_PITCH_REF_KP * INS.Pitch -
            JUMP_RETRACT_WHEEL_PITCH_REF_KD * INS.Gyro[1] +
            normal_airborne_pitch_ref_integral_,
        JUMP_RETRACT_WHEEL_PITCH_REF_MAX);
    const float airborne_wheel_ref_l =
        normal_airborne_wheel_ref_l_ + pitch_speed_ref_trim;
    const float airborne_wheel_ref_r =
        normal_airborne_wheel_ref_r_ + pitch_speed_ref_trim;
    const float left_error =
        left_wheel.Get_Now_Omega() - airborne_wheel_ref_l;
    const float right_error =
        right_wheel.Get_Now_Omega() - airborne_wheel_ref_r;

    normal_airborne_wheel_integral_l_ = ClampAbs(
        normal_airborne_wheel_integral_l_ +
            JUMP_WHEEL_SPEED_KI * left_error * airborne_dt,
        JUMP_WHEEL_INTEGRAL_MAX);
    normal_airborne_wheel_integral_r_ = ClampAbs(
        normal_airborne_wheel_integral_r_ -
            JUMP_WHEEL_SPEED_KI * right_error * airborne_dt,
        JUMP_WHEEL_INTEGRAL_MAX);
    l_wheel_T_ = ClampAbs(JUMP_WHEEL_SPEED_KP * left_error +
                              normal_airborne_wheel_integral_l_,
                          JUMP_WHEEL_TORQUE_MAX);
    r_wheel_T_ = ClampAbs(-JUMP_WHEEL_SPEED_KP * right_error +
                              normal_airborne_wheel_integral_r_,
                          JUMP_WHEEL_TORQUE_MAX);
  } else if (normal_airborne_wheel_control_active_) {
    normal_airborne_wheel_control_active_ = false;
    normal_airborne_wheel_integral_l_ = 0.0f;
    normal_airborne_wheel_integral_r_ = 0.0f;
    normal_airborne_pitch_ref_integral_ = 0.0f;
  }
  if (jump_landing_handoff) {
    l_wheel_T_ *= jump_landing_wheel_scale;
    r_wheel_T_ *= jump_landing_wheel_scale;
  }
  const bool leso_enable =
#if LESO_COMPENSATION_ENABLE
      !GetOffGround() && !normal_airborne_seen_ && !normal_touchdown_active_ &&
      jump_normal_handoff_count_ == 0U && controller_dt_ > 0.0f &&
      controller_dt_ < 0.02f && GetDmFaultMaskNow() == 0U &&
      GetDmOfflineMaskNow() == 0U && GetDmDisabledMaskNow() == 0U;
#else
      false;
#endif
  ApplyLesoCompensation(leso_enable);
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

void balance_Chassis::EnableAllJointMotors() {
  lf_joint_.Enable();
  lb_joint_.Enable();
  rf_joint_.Enable();
  rb_joint_.Enable();
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
  const bool jump_landing_phase =
      current_status == JUMP_LAND_PREP ||
      current_status == JUMP_LAND_IMPACT ||
      current_status == JUMP_LAND_SETTLE;
  const bool jump_contact_detection_enabled =
      jump_landing_phase && jump_airborne_confirmed_;
  const bool jump_left_force_contact =
      jump_contact_detection_enabled &&
      left_leg_.GetForceNormal() > JUMP_LAND_CONTACT_FORCE_THRESHOLD &&
      left_leg_.GetLegSpeed() <= JUMP_LAND_CONTACT_EXTENSION_SPEED_MAX;
  const bool jump_right_force_contact =
      jump_contact_detection_enabled &&
      right_leg_.GetForceNormal() > JUMP_LAND_CONTACT_FORCE_THRESHOLD &&
      right_leg_.GetLegSpeed() <= JUMP_LAND_CONTACT_EXTENSION_SPEED_MAX;

  // A wheel can catch the stair edge without producing a clean 30 N normal
  // force estimate.  Once landing deployment has had time to start, regard a
  // leg as geometrically blocked when it remains well behind the shared length
  // reference and has almost stopped extending.  Each side is qualified
  // independently so the contacted leg can enter impedance control while the
  // other side continues to deploy.
  const bool jump_blocked_detector_armed =
      jump_contact_detection_enabled &&
      jump_timer >= JUMP_LAND_BLOCKED_ARM_TICKS;
  const bool jump_left_blocked_sample =
      jump_blocked_detector_armed &&
      left_leg_.GetForceNormal() > JUMP_LAND_BLOCKED_FORCE_SEED &&
      jump_land_deploy_ref_ - left_leg_.GetLegLen() >=
          JUMP_LAND_BLOCKED_LENGTH_ERROR &&
      left_leg_.GetLegSpeed() <= JUMP_LAND_BLOCKED_SPEED_MAX;
  const bool jump_right_blocked_sample =
      jump_blocked_detector_armed &&
      right_leg_.GetForceNormal() > JUMP_LAND_BLOCKED_FORCE_SEED &&
      jump_land_deploy_ref_ - right_leg_.GetLegLen() >=
          JUMP_LAND_BLOCKED_LENGTH_ERROR &&
      right_leg_.GetLegSpeed() <= JUMP_LAND_BLOCKED_SPEED_MAX;
  if (jump_left_blocked_sample) {
    if (jump_left_blocked_count_ < JUMP_LAND_BLOCKED_CONFIRM_TICKS)
      jump_left_blocked_count_++;
  } else {
    jump_left_blocked_count_ = 0U;
  }
  if (jump_right_blocked_sample) {
    if (jump_right_blocked_count_ < JUMP_LAND_BLOCKED_CONFIRM_TICKS)
      jump_right_blocked_count_++;
  } else {
    jump_right_blocked_count_ = 0U;
  }
  if (jump_left_blocked_count_ >= JUMP_LAND_BLOCKED_CONFIRM_TICKS)
    jump_left_blocked_contact_ = true;
  if (jump_right_blocked_count_ >= JUMP_LAND_BLOCKED_CONFIRM_TICKS)
    jump_right_blocked_contact_ = true;
  const bool jump_left_contact_raw =
      jump_landing_phase
          ? (jump_left_force_contact || jump_left_blocked_contact_)
          : left_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
  const bool jump_right_contact_raw =
      jump_landing_phase
          ? (jump_right_force_contact || jump_right_blocked_contact_)
          : right_leg_.GetForceNormal() > OFF_GROUND_EXIT_THRESHOLD;
  if (jump_landing_phase && jump_airborne_confirmed_) {
    // Capture each leg at the first raw impact.  The captured length becomes
    // that side's zero-step impedance reference after contact is confirmed.
    if (jump_left_contact_raw && !jump_touchdown_capture_l_) {
      jump_touchdown_capture_l_ = true;
      jump_touchdown_ref_l_ = left_leg_.GetLegLen();
    }
    if (jump_right_contact_raw && !jump_touchdown_capture_r_) {
      jump_touchdown_capture_r_ = true;
      jump_touchdown_ref_r_ = right_leg_.GetLegLen();
    }
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
  if (jump_landing_phase && jump_both_contact_latched_ &&
      !jump_landing_pitch_ready_) {
    // Do not immediately brake the translation reference to zero while the
    // body still has a large landing pitch transient.  Requiring consecutive
    // quiet samples prevents a single IMU zero crossing from releasing the
    // second-stage brake too early.
    const bool pitch_ready_sample =
        fabsf(INS.Pitch) <= JUMP_LAND_BRAKE_PITCH_READY_ANGLE &&
        fabsf(INS.Gyro[1]) <= JUMP_LAND_BRAKE_PITCH_READY_RATE;
    if (pitch_ready_sample) {
      if (jump_landing_pitch_ready_count_ <
          JUMP_LAND_BRAKE_PITCH_READY_TICKS)
        jump_landing_pitch_ready_count_++;
    } else {
      jump_landing_pitch_ready_count_ = 0U;
    }
    if (jump_landing_pitch_ready_count_ >=
        JUMP_LAND_BRAKE_PITCH_READY_TICKS)
      jump_landing_pitch_ready_ = true;
  } else if (!jump_both_contact_latched_) {
    jump_landing_pitch_ready_count_ = 0U;
  }

  float left_ref = MIN_LEG_LENGTH;
  float right_ref = MIN_LEG_LENGTH;
  float left_ff = 0.0f;
  float right_ff = 0.0f;
  float direct_leg_force_l = 0.0f;
  float direct_leg_force_r = 0.0f;
  bool override_leg_force_l = false;
  bool override_leg_force_r = false;
  bool use_landing_leg_force = false;

  if (current_status == JUMP_ASCEND || current_status == JUMP_RETRACT ||
      jump_landing_phase) {
    jump_timer++;
    const bool jump_raw_unloaded =
        (left_leg_.GetForceNormal() < OFF_GROUND_ENTER_THRESHOLD) &&
        (right_leg_.GetForceNormal() < OFF_GROUND_ENTER_THRESHOLD);
    if (!jump_airborne_confirmed_) {
      if (jump_raw_unloaded) {
        if (jump_airborne_confirm_count_ < JUMP_AIRBORNE_CONFIRM_TICKS)
          jump_airborne_confirm_count_++;
      } else {
        jump_airborne_confirm_count_ = 0U;
      }
      if (jump_airborne_confirm_count_ >= JUMP_AIRBORNE_CONFIRM_TICKS ||
          GetOffGround()) {
        jump_airborne_confirmed_ = true;
      }
    }
    if (jump_airborne_confirmed_) {
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
    direct_leg_force_l = k_jump_force;
    direct_leg_force_r = k_jump_force;
    override_leg_force_l = true;
    override_leg_force_r = true;

    // Each leg latches independently after it reaches 0.40 m.  Rebound or a
    // small left/right timing difference must not clear the other side's
    // arrival, otherwise ASCEND can remain active indefinitely.
    if (left_leg_.GetLegLen() >= JUMP_EXTEND_READY_LENGTH)
      jump_extend_ready_l_ = true;
    if (right_leg_.GetLegLen() >= JUMP_EXTEND_READY_LENGTH)
      jump_extend_ready_r_ = true;
    // No fixed-time escape is used: both legs must actually have reached the
    // requested extension threshold before retract begins.
    if (jump_extend_ready_l_ && jump_extend_ready_r_) {
      next_status = JUMP_RETRACT;
    }
    break;

  case JUMP_RETRACT: {
    // Stage 3: retract each airborne leg independently.  If one leg reaches a
    // stair first, hold that side compliantly and keep retracting the other;
    // a unilateral contact must not terminate RETRACT.
    left_ref = JUMP_RETRACT_LENGTH;
    right_ref = JUMP_RETRACT_LENGTH;

    const bool left_short =
        left_leg_.GetLegLen() <=
        (JUMP_RETRACT_LENGTH + JUMP_RETRACT_LENGTH_TOLERANCE);
    const bool right_short =
        right_leg_.GetLegLen() <=
        (JUMP_RETRACT_LENGTH + JUMP_RETRACT_LENGTH_TOLERANCE);
    // Launch-force estimation remains high for several milliseconds after
    // wheel lift-off. Do not interpret that transient as stair contact until
    // both legs have been reliably unloaded and RETRACT has run for 40 ms.
    const bool retract_contact_allowed =
        jump_airborne_confirmed_ &&
        jump_timer >= JUMP_RETRACT_CONTACT_BLANK_TICKS;
    const bool left_retract_contact =
        retract_contact_allowed && jump_left_contact_raw;
    const bool right_retract_contact =
        retract_contact_allowed && jump_right_contact_raw;

    if (left_retract_contact && !jump_touchdown_capture_l_) {
      jump_touchdown_capture_l_ = true;
      jump_touchdown_ref_l_ = left_leg_.GetLegLen();
    }
    if (right_retract_contact && !jump_touchdown_capture_r_) {
      jump_touchdown_capture_r_ = true;
      jump_touchdown_ref_r_ = right_leg_.GetLegLen();
    }

    if (left_retract_contact) {
      direct_leg_force_l = CalculateLandingImpedanceForce(
          jump_touchdown_ref_l_, left_leg_.GetLegLen(),
          left_leg_.GetLegSpeed(),
          k_gravity_comp * arm_cos_f32(left_leg_.GetTheta()),
          JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
          JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
          JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
          JUMP_LAND_SOFT_STOP_KD, true);
      override_leg_force_l = true;
    } else if (!left_short) {
      direct_leg_force_l = left_leg_.GetLegLen() > k_retract_near_length
                               ? k_retract_fast_force
                               : k_retract_near_force;
      override_leg_force_l = true;
    }

    if (right_retract_contact) {
      direct_leg_force_r = CalculateLandingImpedanceForce(
          jump_touchdown_ref_r_, right_leg_.GetLegLen(),
          right_leg_.GetLegSpeed(),
          k_gravity_comp * arm_cos_f32(right_leg_.GetTheta()),
          JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
          JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
          JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
          JUMP_LAND_SOFT_STOP_KD, true);
      override_leg_force_r = true;
    } else if (!right_short) {
      direct_leg_force_r = right_leg_.GetLegLen() > k_retract_near_length
                                ? k_retract_fast_force
                                : k_retract_near_force;
      override_leg_force_r = true;
    }

    const bool left_retract_resolved = left_short || left_retract_contact;
    const bool right_retract_resolved = right_short || right_retract_contact;
    if (left_retract_resolved && right_retract_resolved) {
      if (jump_length_ready_count_ < JUMP_LENGTH_READY_TICKS) {
        jump_length_ready_count_++;
      }
    } else {
      jump_length_ready_count_ = 0;
    }

    // No extra airborne hold after retraction. Keep only the short arrival
    // confirmation used to reject a single noisy length sample, then hand the
    // confirmed airborne robot directly to NORMAL. ChangeState() preserves the
    // off-ground flag and suppresses the grounded 250 ms jump hand-off; NORMAL
    // therefore deploys the legs with its own off-ground controller and uses
    // its existing touchdown detector when support returns.
    if (jump_length_ready_count_ >= JUMP_LENGTH_READY_TICKS &&
        jump_liftoff_seen_) {
      next_status = JUMP_NONE;
    }
    break;
  }

  case JUMP_LAND_PREP: {
    // Airborne deployment only.  The reference is continuous across all
    // landing substates so a first contact cannot create a length command step.
    jump_land_deploy_ref_ = SlewTowards(
        jump_land_deploy_ref_, JUMP_LAND_PREP_LENGTH,
        JUMP_LAND_PREP_SLEW_PER_TICK);
    left_ref = jump_land_deploy_ref_;
    right_ref = jump_land_deploy_ref_;
    left_leg_F_ = ClampRange(
        JUMP_LAND_PREP_KP * (left_ref - left_leg_.GetLegLen()) -
            JUMP_LAND_PREP_KD * left_leg_.GetLegSpeed(),
        JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    right_leg_F_ = ClampRange(
        JUMP_LAND_PREP_KP * (right_ref - right_leg_.GetLegLen()) -
            JUMP_LAND_PREP_KD * right_leg_.GetLegSpeed(),
        JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    use_landing_leg_force = true;
    if (jump_left_contact_raw || jump_right_contact_raw)
      next_status = JUMP_LAND_IMPACT;
    break;
  }

  case JUMP_LAND_IMPACT: {
    // Absorb the first-contact leg independently.  The other leg keeps
    // deploying toward the protected geometry until it reaches the ground.
    jump_land_deploy_ref_ = SlewTowards(
        jump_land_deploy_ref_, JUMP_LAND_PREP_LENGTH,
        JUMP_LAND_PREP_SLEW_PER_TICK);
    const bool left_impact = jump_left_contact_raw || jump_left_supported;
    const bool right_impact = jump_right_contact_raw || jump_right_supported;
    left_ref = left_impact && jump_touchdown_capture_l_
                   ? jump_touchdown_ref_l_
                   : jump_land_deploy_ref_;
    right_ref = right_impact && jump_touchdown_capture_r_
                    ? jump_touchdown_ref_r_
                    : jump_land_deploy_ref_;
    if (left_impact) {
      left_leg_F_ = CalculateLandingImpedanceForce(
          left_ref, left_leg_.GetLegLen(), left_leg_.GetLegSpeed(),
          k_gravity_comp * arm_cos_f32(left_leg_.GetTheta()),
          JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
          JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
          JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
          JUMP_LAND_SOFT_STOP_KD, true);
    } else {
      left_leg_F_ = ClampRange(
          JUMP_LAND_PREP_KP * (left_ref - left_leg_.GetLegLen()) -
              JUMP_LAND_PREP_KD * left_leg_.GetLegSpeed(),
          JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    }
    if (right_impact) {
      right_leg_F_ = CalculateLandingImpedanceForce(
          right_ref, right_leg_.GetLegLen(), right_leg_.GetLegSpeed(),
          k_gravity_comp * arm_cos_f32(right_leg_.GetTheta()),
          JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
          JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
          JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
          JUMP_LAND_SOFT_STOP_KD, true);
    } else {
      right_leg_F_ = ClampRange(
          JUMP_LAND_PREP_KP * (right_ref - right_leg_.GetLegLen()) -
              JUMP_LAND_PREP_KD * right_leg_.GetLegSpeed(),
          JUMP_LAND_PREP_FORCE_MIN, JUMP_LAND_PREP_FORCE_MAX);
    }
    use_landing_leg_force = true;
    if (jump_both_supported)
      next_status = JUMP_LAND_SETTLE;
    break;
  }

  case JUMP_LAND_SETTLE: {
    // Both legs now use the same compliant impact absorber as NORMAL's
    // step-down path, but with jump-only gains and independent state storage.
    left_ref = jump_touchdown_capture_l_ ? jump_touchdown_ref_l_
                                         : left_leg_.GetLegLen();
    right_ref = jump_touchdown_capture_r_ ? jump_touchdown_ref_r_
                                           : right_leg_.GetLegLen();
    left_leg_F_ = CalculateLandingImpedanceForce(
        left_ref, left_leg_.GetLegLen(), left_leg_.GetLegSpeed(),
        k_gravity_comp * arm_cos_f32(left_leg_.GetTheta()),
        JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
        JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
        JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
        JUMP_LAND_SOFT_STOP_KD, jump_left_supported);
    right_leg_F_ = CalculateLandingImpedanceForce(
        right_ref, right_leg_.GetLegLen(), right_leg_.GetLegSpeed(),
        k_gravity_comp * arm_cos_f32(right_leg_.GetTheta()),
        JUMP_LAND_IMPACT_KP, JUMP_LAND_IMPACT_KD,
        JUMP_LAND_IMPACT_FORCE_MIN, JUMP_LAND_IMPACT_FORCE_MAX,
        JUMP_LAND_SOFT_LIMIT, JUMP_LAND_SOFT_STOP_KP,
        JUMP_LAND_SOFT_STOP_KD, jump_right_supported);
    use_landing_leg_force = true;

    const float landing_chassis_speed =
        0.5f * (right_wheel.Get_Now_Omega() -
                left_wheel.Get_Now_Omega());
    const bool lengths_safe =
        left_leg_.GetLegLen() >= JUMP_LAND_MIN_SAFE_LENGTH &&
        right_leg_.GetLegLen() >= JUMP_LAND_MIN_SAFE_LENGTH;
    const bool landing_stable =
        jump_both_supported && lengths_safe &&
        fabsf(left_leg_.GetLegSpeed()) <= JUMP_LAND_LEG_SPEED_OK &&
        fabsf(right_leg_.GetLegSpeed()) <= JUMP_LAND_LEG_SPEED_OK &&
        fabsf(INS.Pitch) <= JUMP_LAND_BODY_ANGLE_OK &&
        fabsf(INS.Roll) <= JUMP_LAND_BODY_ANGLE_OK &&
        fabsf(INS.Gyro[1]) <= JUMP_LAND_BODY_RATE_OK &&
        fabsf(INS.Gyro[0]) <= JUMP_LAND_BODY_RATE_OK &&
        fabsf(landing_chassis_speed) <= JUMP_LAND_SETTLE_CHASSIS_SPEED;
    if (jump_timer >= JUMP_LAND_SETTLE_MIN_TICKS && landing_stable) {
      if (jump_landing_ready_count_ < JUMP_LAND_SETTLE_STABLE_TICKS)
        jump_landing_ready_count_++;
    } else {
      jump_landing_ready_count_ = 0U;
    }
    const bool settle_complete =
        jump_landing_ready_count_ >= JUMP_LAND_SETTLE_STABLE_TICKS;
    // Remain in the jump-only landing controller until it is genuinely stable.
    // A fixed timeout previously handed a still-moving robot to NORMAL and
    // could turn the residual pitch correction into a rearward run-off.
    if (jump_liftoff_seen_ && settle_complete)
      next_status = JUMP_NONE;
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
  if (!use_landing_leg_force) {
    LegLenCalc(left_ref, right_ref, left_ff, right_ff);
  }
  // Jump-only per-leg force overrides bypass the filtered length PID. This
  // lets the still-airborne side retract at full force while a contacted side
  // remains compliant instead of ending the whole retract phase.
  if (override_leg_force_l)
    left_leg_F_ = direct_leg_force_l;
  if (override_leg_force_r)
    right_leg_F_ = direct_leg_force_r;

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
    // The waveform shows rearward pitch already accumulating in ASCEND.  As
    // soon as lift-off is confirmed, use the same high-gain/rate-damped hip
    // loop as RETRACT instead of waiting for the length state transition.
    const bool retract_pitch_control =
        current_status == JUMP_RETRACT ||
        (current_status == JUMP_ASCEND && jump_liftoff_seen_);
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
    const float jump_roll_force_limit =
        jump_landing_phase && jump_either_supported
            ? JUMP_LAND_ROLL_FORCE_MAX
            : JUMP_ROLL_FORCE_MAX;
    const float jump_roll_force = ClampAbs(
        -JUMP_ROLL_KP * INS.Roll - JUMP_ROLL_KD * INS.Gyro[0] +
            jump_roll_integral_,
        jump_roll_force_limit);
    left_leg_F_ += jump_roll_force;
    right_leg_F_ -= jump_roll_force;

    // Once airborne, place both wheels ahead of or behind the chassis according
    // to the forward speed captured at take-off. VMC defines
    // theta = phi0 - pi/2 + body_pitch, therefore
    // phi0_ref = pi/2 - body_pitch + theta_ref.
    const bool airborne_world_angle_control =
        jump_liftoff_seen_ || jump_landing_phase;
    if (airborne_world_angle_control) {
      const float airborne_theta_goal = ClampAbs(
          AIRBORNE_LEG_FORWARD_SIGN * AIRBORNE_LEG_SPEED_ANGLE_GAIN *
              jump_airborne_forward_speed_ref_,
          AIRBORNE_LEG_ANGLE_MAX);
      jump_airborne_theta_ref_ = SlewTowards(
          jump_airborne_theta_ref_, airborne_theta_goal,
          AIRBORNE_LEG_ANGLE_SLEW_PER_TICK);
      const float airborne_phi0_target =
          JUMP_PHI0_TARGET - INS.Pitch + jump_airborne_theta_ref_;
      jump_land_phi0_ref_l_ = airborne_phi0_target;
      jump_land_phi0_ref_r_ = airborne_phi0_target;
    } else {
      jump_airborne_theta_ref_ = SlewTowards(
          jump_airborne_theta_ref_, 0.0f,
          AIRBORNE_LEG_ANGLE_SLEW_PER_TICK);
    }
    const float left_phi0_target =
        airborne_world_angle_control ? jump_land_phi0_ref_l_
                                     : JUMP_PHI0_TARGET;
    const float right_phi0_target =
        airborne_world_angle_control ? jump_land_phi0_ref_r_
                                     : JUMP_PHI0_TARGET;
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
    // In flight that is world-frame theta_dot = phi0_dot + pitch_rate.
    const float left_phi0_rate =
        left_leg_.GetPhi0Speed() +
        (airborne_world_angle_control ? INS.Gyro[1] : 0.0f);
    const float right_phi0_rate =
        right_leg_.GetPhi0Speed() +
        (airborne_world_angle_control ? INS.Gyro[1] : 0.0f);
    // Do not superimpose body-pitch hip torque on the world-frame foot-placement
    // coordinate. Reaction-wheel control retains airborne pitch authority.
    const float hip_pitch_torque =
        airborne_world_angle_control ? 0.0f : jump_pitch_torque;
    const float phi0_kp = airborne_world_angle_control
                              ? AIRBORNE_LEG_ANGLE_KP
                              : JUMP_PHI0_KP;
    const float phi0_kd = airborne_world_angle_control
                              ? AIRBORNE_LEG_ANGLE_KD
                              : JUMP_PHI0_KD;
    const float hip_torque_limit = airborne_world_angle_control
                                       ? AIRBORNE_LEG_ANGLE_TORQUE_MAX
                                       : JUMP_HIP_TORQUE_TOTAL_MAX;
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
      jump_landing_phase) {
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
    // pitch loop a common motor-shaft reference trim. The follow-up waveform
    // showed that the reversed sign drove both shafts negative together with
    // the negative (rearward) pitch, so restore the reaction-wheel direction:
    // negative pitch commands positive common shaft acceleration.
    // Retraction and the still-airborne part of landing deployment both create
    // large internal leg impulses while the hips are unavailable for body
    // pitch correction. Keep the stronger, rate-damped wheel loop active until
    // the first leg has genuinely made contact.
    const bool retract_wheel_pitch_control =
        current_status == JUMP_RETRACT ||
        (jump_landing_phase && !jump_either_supported);
    const float wheel_pitch_ref_kp =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_KP
                                    : JUMP_WHEEL_PITCH_REF_KP;
    const float wheel_pitch_ref_kd =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_KD
                                    : JUMP_WHEEL_PITCH_REF_KD;
    const float wheel_pitch_ref_max =
        retract_wheel_pitch_control ? JUMP_RETRACT_WHEEL_PITCH_REF_MAX
                                    : JUMP_WHEEL_PITCH_REF_MAX;
    if (retract_wheel_pitch_control) {
      jump_wheel_pitch_ref_integral_ = ClampAbs(
          jump_wheel_pitch_ref_integral_ -
              JUMP_RETRACT_WHEEL_PITCH_REF_KI * INS.Pitch * jump_dt,
          JUMP_RETRACT_WHEEL_PITCH_REF_I_MAX);
    } else {
      jump_wheel_pitch_ref_integral_ *= 0.80f;
    }
    float pitch_speed_ref_trim = ClampAbs(
        -wheel_pitch_ref_kp * INS.Pitch -
            wheel_pitch_ref_kd * INS.Gyro[1] +
            jump_wheel_pitch_ref_integral_,
        wheel_pitch_ref_max);
    if (current_status == JUMP_ASCEND) {
      // The 2 -> 1 edge captures both shaft speeds. During powered extension
      // hold those values exactly; pitch correction starts only after ASCEND.
      pitch_speed_ref_trim = 0.0f;
      jump_wheel_pitch_ref_integral_ = 0.0f;
    }
    float captured_speed_scale = 1.0f;
    float wheel_torque_limit = JUMP_WHEEL_TORQUE_MAX;
    if (jump_landing_phase) {
      // LAND_PREP never accelerates above the captured 2 -> 1 wheel speed.
      // Late in flight, pre-spin the wheels down modestly; raw first contact
      // starts a light brake immediately, while debounced contact permits a
      // stronger reduction.  Full braking still waits for bilateral support.
      float landing_speed_scale_target = 1.0f;
      float landing_speed_slew = JUMP_LAND_FLIGHT_SPEED_SLEW;
      if (jump_both_contact_latched_) {
        landing_speed_scale_target = jump_landing_pitch_ready_
                                         ? 0.0f
                                         : JUMP_LAND_FORWARD_HOLD_SCALE;
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
    if (jump_landing_phase && jump_both_contact_latched_) {
      // Stage 1 retains a small part of the captured forward reference while
      // pitch settles. Stage 2 slews to zero. If touchdown has already slowed
      // a wheel below the hold target, never accelerate it back up.
      float brake_target_l = jump_landing_pitch_ready_
                                 ? 0.0f
                                 : JUMP_LAND_FORWARD_HOLD_SCALE *
                                       jump_wheel_speed_ref_l_;
      float brake_target_r = jump_landing_pitch_ready_
                                 ? 0.0f
                                 : JUMP_LAND_FORWARD_HOLD_SCALE *
                                       jump_wheel_speed_ref_r_;
      if (!jump_landing_pitch_ready_) {
        if (jump_wheel_speed_ref_l_ > 0.0f &&
            jump_landing_brake_ref_l_ < brake_target_l)
          brake_target_l = jump_landing_brake_ref_l_;
        if (jump_wheel_speed_ref_l_ < 0.0f &&
            jump_landing_brake_ref_l_ > brake_target_l)
          brake_target_l = jump_landing_brake_ref_l_;
        if (jump_wheel_speed_ref_r_ > 0.0f &&
            jump_landing_brake_ref_r_ < brake_target_r)
          brake_target_r = jump_landing_brake_ref_r_;
        if (jump_wheel_speed_ref_r_ < 0.0f &&
            jump_landing_brake_ref_r_ > brake_target_r)
          brake_target_r = jump_landing_brake_ref_r_;
      }
      jump_landing_brake_ref_l_ =
          SlewTowards(jump_landing_brake_ref_l_, brake_target_l,
                      JUMP_LAND_BRAKE_REF_SLEW);
      jump_landing_brake_ref_r_ =
          SlewTowards(jump_landing_brake_ref_r_, brake_target_r,
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
          jump_landing_pitch_ready_ &&
          fabsf(jump_wheel_speed_ref_l_) >
              JUMP_LAND_BRAKE_ZERO_CROSS_SPEED &&
          left_omega * jump_wheel_speed_ref_l_ <= 0.0f;
      const bool right_zero_crossed_now =
          jump_landing_pitch_ready_ &&
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
    if (jump_landing_phase) {
      // Once ground contact is possible, pitch correction may reduce the
      // captured reference but must not command reverse travel. In RETRACT the
      // wheels are airborne, so allowing a shaft reference to cross zero is
      // necessary to preserve full reaction-wheel pitch authority.
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
    if (jump_landing_phase && jump_both_contact_latched_) {
      jump_wheel_integral_l_ = 0.0f;
      jump_wheel_integral_r_ = 0.0f;
      jump_wheel_pitch_ref_integral_ = 0.0f;
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
      // Extension produces a repeatable positive common shaft acceleration
      // and a simultaneous negative body-pitch impulse.  Waiting for a speed
      // error makes the loop reach its limit only after that impulse has
      // already rotated the body.  Apply an ASCEND-only common braking
      // feed-forward while keeping both captured speed references unchanged.
      // SetMotorTor() negates the left internal command, hence the opposite
      // internal signs below produce the same negative physical motor torque.
      const float ascend_wheel_hold_ff =
          current_status == JUMP_ASCEND ? JUMP_ASCEND_WHEEL_HOLD_FF : 0.0f;
      l_wheel_T_ = ClampAbs(JUMP_WHEEL_SPEED_KP * left_speed_error +
                                jump_wheel_integral_l_ +
                                ascend_wheel_hold_ff,
                            wheel_torque_limit);
      r_wheel_T_ = ClampAbs(-JUMP_WHEEL_SPEED_KP * right_speed_error +
                                jump_wheel_integral_r_ -
                                ascend_wheel_hold_ff,
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
      jump_wheel_pitch_ref_integral_ = 0.0f;
      jump_landing_speed_scale_ = 1.0f;
      jump_landing_zero_cross_l_ = false;
      jump_landing_zero_cross_r_ = false;
      jump_landing_pitch_ready_count_ = 0U;
      jump_landing_pitch_ready_ = false;
      jump_left_contact_count_ = jump_right_contact_count_ = 0U;
      jump_left_release_count_ = jump_right_release_count_ = 0U;
      jump_left_blocked_count_ = jump_right_blocked_count_ = 0U;
      jump_left_contact_ = jump_right_contact_ = false;
      jump_left_blocked_contact_ = jump_right_blocked_contact_ = false;
      jump_both_contact_latched_ = false;
      jump_touchdown_capture_l_ = false;
      jump_touchdown_capture_r_ = false;
      jump_touchdown_ref_l_ = left_leg_.GetLegLen();
      jump_touchdown_ref_r_ = right_leg_.GetLegLen();
      jump_land_deploy_ref_ = ClampRange(
          0.5f * (left_leg_.GetLegLen() + right_leg_.GetLegLen()),
          JUMP_RETRACT_LENGTH, JUMP_LAND_PREP_LENGTH);
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

// ---------- 上台阶完整摆腿轨迹（让空间→收腿→顶腿 合并为一条连续曲线）----------
// 由「全过程-优化6-顶腿.xlsx」动作段(源 idx2-422 共 421 点) 移动平均(窗5)平滑 + 线性重采样为 101 点，
// 映射到 STEP_UP_DURATION 时长，StepUpTrajAt 线性插值回放。
// φ0：1.595(轮在髋下) → 峰值 3.115(收腿后摆) → 1.539(回落)；l0：0.3656 → 峰值 0.4119(让空间) → 谷值 0.1320(收腿最短) → 0.1350(收腿保持)。
static const float kStepUpPhi0Traj[STEP_UP_WAYPOINT_N] = {
    1.59520f, 1.59792f, 1.60595f, 1.61934f, 1.63768f, 1.66056f, 1.68675f, 1.71580f,
    1.75229f, 1.79399f, 1.83678f, 1.87903f, 1.92047f, 1.96076f, 2.00041f, 2.04109f,
    2.08344f, 2.12772f, 2.17283f, 2.21797f, 2.26320f, 2.30842f, 2.35319f, 2.39727f,
    2.44032f, 2.48218f, 2.52366f, 2.56489f, 2.60574f, 2.64635f, 2.68717f, 2.72822f,
    2.76907f, 2.80991f, 2.85086f, 2.89135f, 2.93026f, 2.96654f, 3.00080f, 3.03315f,
    3.06191f, 3.08394f, 3.09913f, 3.10893f, 3.11378f, 3.11472f, 3.11459f, 3.11296f,
    3.10382f, 3.08791f, 3.06900f, 3.04550f, 3.01797f, 2.98784f, 2.95530f, 2.92037f,
    2.88469f, 2.84831f, 2.81024f, 2.77072f, 2.73036f, 2.68896f, 2.64794f, 2.60741f,
    2.57191f, 2.54466f, 2.52716f, 2.51781f, 2.51566f, 2.51570f, 2.51555f, 2.51553f,
    2.51529f, 2.51308f, 2.50355f, 2.48686f, 2.46048f, 2.42608f, 2.39243f, 2.35590f,
    2.31661f, 2.27397f, 2.22996f, 2.18435f, 2.13640f, 2.08601f, 2.03423f, 1.98240f,
    1.93094f, 1.88081f, 1.83272f, 1.78667f, 1.74369f, 1.70400f, 1.66826f, 1.63579f,
    1.60681f, 1.58135f, 1.56061f, 1.54556f, 1.53859f,
};

static const float kStepUpL0Traj[STEP_UP_WAYPOINT_N] = {
    0.36561f, 0.36628f, 0.36822f, 0.37142f, 0.37575f, 0.38102f, 0.38687f, 0.39288f,
    0.39821f, 0.40274f, 0.40648f, 0.40930f, 0.41098f, 0.41158f, 0.41164f, 0.41174f,
    0.41185f, 0.41192f, 0.41192f, 0.41183f, 0.41171f, 0.41161f, 0.41154f, 0.41148f,
    0.41143f, 0.41131f, 0.41118f, 0.41109f, 0.41095f, 0.41086f, 0.41074f, 0.41063f,
    0.41037f, 0.40987f, 0.40901f, 0.40778f, 0.40624f, 0.40447f, 0.40245f, 0.40007f,
    0.39712f, 0.39370f, 0.39040f, 0.38828f, 0.38722f, 0.38699f, 0.38697f, 0.38663f,
    0.38461f, 0.38070f, 0.37409f, 0.36375f, 0.35026f, 0.33428f, 0.31629f, 0.29734f,
    0.27804f, 0.25875f, 0.23987f, 0.22177f, 0.20461f, 0.18878f, 0.17545f, 0.16481f,
    0.15623f, 0.14984f, 0.14585f, 0.14376f, 0.14328f, 0.14331f, 0.14329f, 0.14330f,
    0.14327f, 0.14276f, 0.14068f, 0.13715f, 0.13393f, 0.13342f, 0.13349f, 0.13344f,
    0.13340f, 0.13333f, 0.13310f, 0.13263f, 0.13215f, 0.13201f, 0.13208f, 0.13218f,
    0.13233f, 0.13231f, 0.13231f, 0.13232f, 0.13237f, 0.13245f, 0.13268f, 0.13309f,
    0.13371f, 0.13430f, 0.13476f, 0.13493f, 0.13502f,
};

// 轨迹查表（线性插值）：tick ∈ [0, duration)，traj 长 n 点，均匀映射到 duration 时长
static float StepUpTrajAt(const float *traj, uint32_t n, uint32_t duration, uint32_t tick) {
  const float u = (float)tick / (float)(duration - 1U);
  const float x = u * (float)(n - 1);
  int32_t i = (int32_t)x;
  if (i < 0) i = 0;
  if (i >= (int32_t)(n - 1)) {
    return traj[n - 1];
  }
  const float f = x - (float)i;
  return traj[i] * (1.0f - f) + traj[i + 1] * f;
}

/**
 * @brief 重置上台阶子状态机
 *
 */
void balance_Chassis::ResetStepUpState(){
    step_up_status = STEP_UP_NONE;
    step_up_timer = 0;
    step_up_impact_cnt_ = 0;
    step_up_state_ = false;
}

//上台阶完成的判断函数
bool balance_Chassis::IsStepUpComplete() {
    return step_up_status == STEP_UP_FINISH;//上台阶子状态切换到FINISH时判断上台阶行为结束
}

//上台阶到位判据：左右腿的摆角与腿长都进入目标容差内即判定到位
bool balance_Chassis::StepUpTargetReached(float target_phi0, float target_l0) {
  return fabsf(left_leg_.GetPhi0() - target_phi0) < STEP_UP_POS_ANGLE_TOL &&
         fabsf(left_leg_.GetLegLen() - target_l0) < STEP_UP_POS_LEN_TOL &&
         fabsf(right_leg_.GetPhi0() - target_phi0) < STEP_UP_POS_ANGLE_TOL &&
         fabsf(right_leg_.GetLegLen() - target_l0) < STEP_UP_POS_LEN_TOL;
}

//上台阶撞击检测：使能开关(step_up_flag==2)打开 + 双腿已抬高到安全高度 + 前进中，
//触发三信号（俯仰抖动 / 轮速骤降 / 轮扭矩）任一命中即去抖确认，判定撞击（断开 LQR 进入固定动作）。
bool balance_Chassis::IsStepUpImpactDetected() {
  // 每拍更新编码器中心轮速与帧间减速量（供判据与上位机标定使用，正=减速）
  const float encoder_center_speed =
      0.5f * (-left_wheel.Get_Now_Omega() + right_wheel.Get_Now_Omega()) *
      k_wheel_radius;
  step_up_wheel_decel_ = step_up_last_encoder_speed_ - encoder_center_speed;
  step_up_last_encoder_speed_ = encoder_center_speed;

  // 使能开关未打开，或不在 NORMAL，不检测
  if ((uint8_t)sbus_rx_data.step_up_flag != 2 ||
      robot_status != STATE_NORMAL) {
    step_up_impact_cnt_ = 0;
    return false;
  }
  // 双腿必须已抬高到安全高度（操作者主动抬高，作为预置条件）
  if (left_leg_.GetLegLen() < STEP_UP_SAFE_LEN ||
      right_leg_.GetLegLen() < STEP_UP_SAFE_LEN) {
    step_up_impact_cnt_ = 0;
    return false;
  }
  // 前进中才检测（静止/后退不触发）
  if (vel_ < STEP_UP_IMPACT_MIN_SPEED) {
    step_up_impact_cnt_ = 0;
    return false;
  }

  // 触发信号（放宽：三选一命中即累计去抖；前置条件已滤掉大部分误触）
  // 1) 抖动：俯仰角速度尖峰，撞击硬冲击直接反映在 IMU，无卡尔曼抹平
  const bool pitch_jolt = fabsf(INS.Gyro[1]) > STEP_UP_IMPACT_GYRO_TH;
  // 2) 速度差：轮速骤降（堵转）。vel_ 由轮速推出、堵转时也会掉，故改用编码器帧间减速度，
  //    它直接反映轮子被别停，量值远大于正常驾驶减速。
  const bool wheel_stalled = step_up_wheel_decel_ > STEP_UP_IMPACT_STALL_DECEL_TH;
  // 3) 扭矩：轮反馈扭矩之和（力矩模式反馈≈指令，较弱，作兜底）
  const float wheel_torque =
      fabsf(left_wheel.Get_Now_Torque()) + fabsf(right_wheel.Get_Now_Torque());
  const bool torque_spike = wheel_torque > STEP_UP_IMPACT_TORQUE_TH;

  if (pitch_jolt || wheel_stalled || torque_spike) {
    if (step_up_impact_cnt_ < STEP_UP_IMPACT_CONFIRM_TICKS) {
      step_up_impact_cnt_++;
    }
    return step_up_impact_cnt_ >= STEP_UP_IMPACT_CONFIRM_TICKS;
  }
  step_up_impact_cnt_ = 0;
  return false;
}

/**
 * @brief 上台阶控制（也要负责状态切换）
 *
 */
void balance_Chassis::StepUpCalc() {
    // 读取当前子状态
    const StepUpStatus current_status = step_up_status;  // 保存当前这一拍执行时的上台阶子状态。
    StepUpStatus next_status = current_status;           // 默认下一状态和当前状态一样，满足条件时在最后切换
    // 固定动作目标（摆角 phi0 + 腿长 l0），按子状态设定
    float target_phi0 = 0.0f;
    float target_l0 = 0.0f;

    // 单一连续轨迹回放：让空间→收腿→展腿 已合并为一条曲线，不再分子状态切段
    switch (current_status) {
      case STEP_UP_CLEAR_LEG:  // 复用为「正在播放」子状态，播完即完成
        target_phi0 = StepUpTrajAt(kStepUpPhi0Traj, STEP_UP_WAYPOINT_N,
                                   STEP_UP_DURATION, step_up_timer);
        target_l0 = StepUpTrajAt(kStepUpL0Traj, STEP_UP_WAYPOINT_N,
                                 STEP_UP_DURATION, step_up_timer);
        step_up_timer++;
        if (step_up_timer >= STEP_UP_DURATION) {
          next_status = STEP_UP_FINISH;
        }
        break;

      case STEP_UP_APPROACH: // 接近阶段已并入 NORMAL，不再单独执行
      case STEP_UP_FINISH:   // 由状态机在 IsStepUpComplete 后切走
      default:
        break;
    }

    // 固定动作：IK + 关节角 LQR → 直接关节力矩（复用 RecoverLegJointLQR 输出链路）
    float l1 = 0.0f, l2 = 0.0f, r1 = 0.0f, r2 = 0.0f;
    RecoverLegJointLQR(left_leg_, target_phi0, target_l0, l1, l2);
    RecoverLegJointLQR(right_leg_, target_phi0, target_l0, r1, r2);
    left_leg_.SetDirectJointTor(l1, l2);
    right_leg_.SetDirectJointTor(r1, r2);

    // 轮速控制：按摆角/腿长两个一次性触发点分段
    //   1) 初始：轮子以固定前进轮速转，靠摩擦力把车体往前顶；
    //   2) 摆角 φ0 首次 > 2.32（后摆到位）：轮子停转（腿悬空/收腿，不再顶）；
    //   3) 腿长 l0 首次 < 0.185（收腿到位）：轮子重新转，直到上台阶动作结束。
    if (!step_up_wheel_stopped_ && target_phi0 > STEP_UP_WHEEL_STOP_PHI0) {
      step_up_wheel_stopped_ = true;  // 摆角第一次越过阈值 → 锁存停转
    }
    if (step_up_wheel_stopped_ && !step_up_wheel_resumed_ && target_l0 < STEP_UP_WHEEL_RESUME_L0) {
      step_up_wheel_resumed_ = true;  // 腿长第一次越过阈值 → 锁存恢复旋转
    }
    const bool wheel_on = !step_up_wheel_stopped_ || step_up_wheel_resumed_;
    if (wheel_on) {
      // 轮速 P 闭环：目标线速度 → 速度误差 → 扭矩（轮子为 M3508 扭矩模式，无内置速度伺服）
      const float left_speed = -left_wheel.Get_Now_Omega() * k_wheel_radius;
      const float right_speed = right_wheel.Get_Now_Omega() * k_wheel_radius;
      l_wheel_T_ = ClampAbs(STEP_UP_WHEEL_SPEED_KP * (STEP_UP_WHEEL_SPEED - left_speed),
                            STEP_UP_WHEEL_TORQUE_MAX);
      r_wheel_T_ = ClampAbs(STEP_UP_WHEEL_SPEED_KP * (STEP_UP_WHEEL_SPEED - right_speed),
                            STEP_UP_WHEEL_TORQUE_MAX);
    } else {
      l_wheel_T_ = 0.0f;
      r_wheel_T_ = 0.0f;
    }

    // 真正切换状态（本拍输出全部生成完之后，再切换到下一子状态）
    if (next_status != current_status) {
      step_up_status = next_status;
      step_up_timer = 0; // 切换状态，清空计时器
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
  // NORMAL uses one longitudinal/pitch/leg controller for the complete command
  // plane.  Yaw is intentionally external to the 4x10 LQR because its yaw-rate
  // columns also command equal-and-opposite hip torques.
  lqr_body_.SetSpeed(normal_mode ? normal_speed_ref_ : target_speed_);
  normal_pivot_pitch_target_ = normal_mode ? kNormalPitchTarget : 0.0f;
  lqr_body_.SetPitchTarget(normal_mode ? normal_pivot_pitch_target_ : 0.0f);
  // Use exactly the same continuously decaying bias as NormalCalc().  At zero
  // input the dedicated midpoint loop owns the zero-speed equilibrium, so LQR
  // must not request a competing +0.02 m/s forward speed.
  const float normal_bias_scale =
      normal_mode
          ? ClampRange(fabsf(normal_speed_ref_) / kNormalSpeedBiasRampSpeed,
                       0.0f, 1.0f)
          : 0.0f;
  const float normal_speed_bias =
      normal_mode ? kNormalSpeedBias * normal_bias_scale
                  : kNormalZeroInputSpeedBias;
  lqr_body_.SetSpeedBias(normal_speed_bias);
  lqr_body_.SetDist(target_dist_);
  // Pivot yaw is handled after LQR in a dedicated differential-wheel loop.
  // Zeroing only the NORMAL pivot yaw states here prevents the same yaw error
  // from also modulating common wheel and leg torques (pitch disturbance).
  lqr_body_.SetRotation(normal_yaw_external ? 0.0f : target_rotation_);
  lqr_body_.SetWRotation(normal_yaw_external ? 0.0f : target_w_rotation_);
  lqr_body_.SetPitchGainScale(normal_mode ? kNormalDrivePitchGainScale : 1.0f);
  // INS.Gyro remains true rad/s. Transform the fitted LQR body-rate gains so
  // the X/Y unit migration does not multiply the established feedback by 57.3.
  lqr_body_.SetBodyRateGainScale(DEGREE_2_RAD);
  // debug模式下，用机身pitch和gyro代替腿部数据
  float theta_l = debug_mode ? INS.Pitch : left_leg_.GetTheta();
  float theta_r = debug_mode ? INS.Pitch : right_leg_.GetTheta();
  float w_theta_l = debug_mode ? INS.Gyro[1] : left_leg_.GetDotTheta();
  float w_theta_r = debug_mode ? INS.Gyro[1] : right_leg_.GetDotTheta();
  if (normal_mode) {
    // LQR uses a zero theta reference. Translate only NORMAL's measurements so
    // that zero error corresponds to phi0 = PI/2 - 0.04 rad. Recovery, jump
    // and joint-debug retain their original leg-angle coordinates.
    const float normal_leg_theta_target =
        kNormalLegPhi0Target - 0.5f * PI;
    theta_l -= normal_leg_theta_target;
    theta_r -= normal_leg_theta_target;
  }
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

void balance_Chassis::ResetLeso() {
  if (leso_active_ || leso_.IsInitialized()) {
    leso_.Reset();
  }
  leso_active_ = false;
  leso_wheel_compensation_gain_ = 0.0f;
  leso_hip_compensation_gain_ = 0.0f;
  leso_wheel_common_disturbance_ = 0.0f;
  leso_hip_common_disturbance_ = 0.0f;
  leso_wheel_disturbance_filtered_ = 0.0f;
  leso_hip_disturbance_filtered_ = 0.0f;
  leso_wheel_correction_ = 0.0f;
  leso_hip_correction_ = 0.0f;
  leso_distance_ = 0.0f;
}

void balance_Chassis::ResetRollLeso() {
  roll_leso_active_ = false;
  roll_leso_z1_ = 0.0f;
  roll_leso_z2_ = 0.0f;
  roll_leso_z3_ = 0.0f;
  roll_leso_applied_force_ = 0.0f;
  roll_leso_disturbance_filtered_ = 0.0f;
  roll_leso_compensation_gain_ = 0.0f;
  roll_leso_correction_ = 0.0f;
}

void balance_Chassis::ResetPitchLeso() {
  pitch_leso_active_ = false;
  pitch_leso_z1_ = 0.0f;
  pitch_leso_z2_ = 0.0f;
  pitch_leso_z3_ = 0.0f;
  pitch_leso_applied_common_torque_ = 0.0f;
  pitch_leso_disturbance_filtered_ = 0.0f;
  pitch_leso_compensation_gain_ = 0.0f;
  pitch_leso_correction_ = 0.0f;
}

float balance_Chassis::UpdatePitchLeso(bool enable, float pitch,
                                       float pitch_rate) {
  if (!enable) {
    ResetPitchLeso();
    return 0.0f;
  }

  if (!pitch_leso_active_) {
    pitch_leso_z1_ = pitch;
    pitch_leso_z2_ = pitch_rate;
    pitch_leso_z3_ = 0.0f;
    pitch_leso_applied_common_torque_ =
        0.5f * (l_wheel_T_ + r_wheel_T_);
    pitch_leso_active_ = true;
    return 0.0f;
  }

  // Third-order linear ESO for the common-wheel-to-Pitch channel. Only the
  // raw Pitch angle drives observer innovation; gyro rate is retained as the
  // LQR damping measurement and as a bumpless observer initial condition.
  const float observer_bandwidth = PITCH_LESO_BANDWIDTH_RAD_PER_SECOND;
  const float beta1 = 3.0f * observer_bandwidth;
  const float beta2 = 3.0f * observer_bandwidth * observer_bandwidth;
  const float beta3 = observer_bandwidth * observer_bandwidth *
                      observer_bandwidth;
  const float innovation = pitch - pitch_leso_z1_;
  const float z1_dot = pitch_leso_z2_ + beta1 * innovation;
  const float z2_dot = pitch_leso_z3_ +
                       PITCH_LESO_INPUT_GAIN *
                           pitch_leso_applied_common_torque_ +
                       beta2 * innovation;
  const float z3_dot = beta3 * innovation;
  pitch_leso_z1_ += controller_dt_ * z1_dot;
  pitch_leso_z2_ += controller_dt_ * z2_dot;
  pitch_leso_z3_ = ClampAbs(
      pitch_leso_z3_ + controller_dt_ * z3_dot,
      PITCH_LESO_DISTURBANCE_ACCEL_LIMIT);

  const bool state_valid =
      pitch_leso_z1_ == pitch_leso_z1_ &&
      pitch_leso_z2_ == pitch_leso_z2_ &&
      pitch_leso_z3_ == pitch_leso_z3_ && fabsf(pitch_leso_z1_) < 1.6f &&
      fabsf(pitch_leso_z2_) < 30.0f &&
      fabsf(pitch_leso_z3_) <= PITCH_LESO_DISTURBANCE_ACCEL_LIMIT;
  if (!state_valid) {
    ResetPitchLeso();
    return 0.0f;
  }

  const float disturbance_lpf_alpha =
      controller_dt_ / (PITCH_LESO_DISTURBANCE_LPF_TAU + controller_dt_);
  pitch_leso_disturbance_filtered_ +=
      disturbance_lpf_alpha *
      (pitch_leso_z3_ - pitch_leso_disturbance_filtered_);
  const float compensation_gain_target = PITCH_LESO_COMPENSATION_TARGET;
  const float compensation_gain_slew =
      (compensation_gain_target >= pitch_leso_compensation_gain_)
          ? PITCH_LESO_COMPENSATION_RAMP_PER_SECOND
          : PITCH_LESO_COMPENSATION_RELEASE_PER_SECOND;
  pitch_leso_compensation_gain_ = SlewTowards(
      pitch_leso_compensation_gain_, compensation_gain_target,
      compensation_gain_slew * controller_dt_);

  const float correction_target = ClampAbs(
      -pitch_leso_compensation_gain_ * pitch_leso_disturbance_filtered_ /
          PITCH_LESO_INPUT_GAIN,
      PITCH_LESO_CORRECTION_LIMIT);
  pitch_leso_correction_ = SlewTowards(
      pitch_leso_correction_, correction_target,
      PITCH_LESO_CORRECTION_SLEW_PER_SECOND * controller_dt_);
  return pitch_leso_correction_;
}

float balance_Chassis::UpdateRollLeso(bool enable, float roll,
                                      float roll_rate) {
  if (!enable) {
    ResetRollLeso();
    return 0.0f;
  }

  if (!roll_leso_active_) {
    roll_leso_z1_ = roll;
    roll_leso_z2_ = roll_rate;
    roll_leso_z3_ = 0.0f;
    roll_leso_applied_force_ = roll_force_cmd_;
    roll_leso_active_ = true;
    return 0.0f;
  }

  // Third-order linear ESO for a second-order roll plant. The gyro remains the
  // damping feedback measurement; the ESO uses Roll as its single output so it
  // estimates one coherent low-frequency acceleration disturbance instead of
  // differentiating gyro noise.
  const float observer_bandwidth = ROLL_LESO_BANDWIDTH_RAD_PER_SECOND;
  const float beta1 = 3.0f * observer_bandwidth;
  const float beta2 = 3.0f * observer_bandwidth * observer_bandwidth;
  const float beta3 = observer_bandwidth * observer_bandwidth *
                      observer_bandwidth;
  const float innovation = roll - roll_leso_z1_;
  const float z1_dot = roll_leso_z2_ + beta1 * innovation;
  const float z2_dot = roll_leso_z3_ +
                       ROLL_LESO_INPUT_GAIN * roll_leso_applied_force_ +
                       beta2 * innovation;
  const float z3_dot = beta3 * innovation;
  roll_leso_z1_ += controller_dt_ * z1_dot;
  roll_leso_z2_ += controller_dt_ * z2_dot;
  roll_leso_z3_ = ClampAbs(roll_leso_z3_ + controller_dt_ * z3_dot,
                           ROLL_LESO_DISTURBANCE_ACCEL_LIMIT);

  // NaN fails the self-equality checks; infinities and divergent finite states
  // fail the explicit bounds. Any numerical fault removes compensation in the
  // same sample and requires a clean reinitialisation on the next sample.
  const bool state_valid =
      roll_leso_z1_ == roll_leso_z1_ && roll_leso_z2_ == roll_leso_z2_ &&
      roll_leso_z3_ == roll_leso_z3_ && fabsf(roll_leso_z1_) < 1.6f &&
      fabsf(roll_leso_z2_) < 30.0f &&
      fabsf(roll_leso_z3_) <= ROLL_LESO_DISTURBANCE_ACCEL_LIMIT;
  if (!state_valid) {
    ResetRollLeso();
    return 0.0f;
  }

  const float disturbance_lpf_alpha =
      controller_dt_ / (ROLL_LESO_DISTURBANCE_LPF_TAU + controller_dt_);
  roll_leso_disturbance_filtered_ +=
      disturbance_lpf_alpha *
      (roll_leso_z3_ - roll_leso_disturbance_filtered_);
  roll_leso_compensation_gain_ +=
      ROLL_LESO_COMPENSATION_RAMP_PER_SECOND * controller_dt_;
  if (roll_leso_compensation_gain_ > ROLL_LESO_COMPENSATION_TARGET) {
    roll_leso_compensation_gain_ = ROLL_LESO_COMPENSATION_TARGET;
  }

  const float correction_target = ClampAbs(
      -roll_leso_compensation_gain_ * roll_leso_disturbance_filtered_ /
          ROLL_LESO_INPUT_GAIN,
      ROLL_LESO_CORRECTION_LIMIT);
  roll_leso_correction_ = SlewTowards(
      roll_leso_correction_, correction_target,
      ROLL_LESO_CORRECTION_SLEW_PER_SECOND * controller_dt_);
  return roll_leso_correction_;
}

void balance_Chassis::ApplyLesoCompensation(bool enable) {
  if (!enable) {
    ResetLeso();
    ResetPitchLeso();
    return;
  }

  // NORMAL deliberately uses encoder-only axle-centre speed because vel_
  // also contains leg-angle and body-pitch rates.  Feeding vel_ to LESO closed
  // a positive loop: leg shake looked like translation, the observer produced
  // a hip disturbance, and the compensation shook the legs again.  Integrate
  // a matching private distance so the observer still preserves x_dot = v.
  if (leso_active_) {
    leso_distance_ += normal_wheel_center_speed_ * controller_dt_;
  }
  const float measurement[LESO_STATE_DIM] = {
      leso_distance_,
      normal_wheel_center_speed_,
      rotation_,
      INS.Gyro[2],
      left_leg_.GetTheta(),
      left_leg_.GetDotTheta(),
      right_leg_.GetTheta(),
      right_leg_.GetDotTheta(),
      INS.Pitch,
      INS.Gyro[1],
  };

  if (!leso_active_) {
    leso_.Reset(measurement);
    if (!leso_.IsInitialized()) {
      ResetLeso();
      return;
    }
    leso_active_ = true;
    leso_wheel_compensation_gain_ = 0.0f;
    leso_hip_compensation_gain_ = 0.0f;
  }

  leso_wheel_common_disturbance_ = leso_.GetWheelCommonDisturbance();
  leso_hip_common_disturbance_ = leso_.GetHipCommonDisturbance();

  const float disturbance_lpf_alpha =
      controller_dt_ / (LESO_DISTURBANCE_LPF_TAU + controller_dt_);
  leso_wheel_disturbance_filtered_ +=
      disturbance_lpf_alpha * (leso_wheel_common_disturbance_ -
                               leso_wheel_disturbance_filtered_);
  leso_hip_disturbance_filtered_ +=
      disturbance_lpf_alpha * (leso_hip_common_disturbance_ -
                               leso_hip_disturbance_filtered_);

  leso_wheel_compensation_gain_ +=
      LESO_COMPENSATION_RAMP_PER_SECOND * controller_dt_;
  if (leso_wheel_compensation_gain_ > LESO_WHEEL_COMPENSATION_TARGET) {
    leso_wheel_compensation_gain_ = LESO_WHEEL_COMPENSATION_TARGET;
  }
  leso_hip_compensation_gain_ +=
      LESO_COMPENSATION_RAMP_PER_SECOND * controller_dt_;
  if (leso_hip_compensation_gain_ > LESO_HIP_COMPENSATION_TARGET) {
    leso_hip_compensation_gain_ = LESO_HIP_COMPENSATION_TARGET;
  }

  // Keep the two matched common channels explicitly allocated: wheel
  // disturbance compensation stays on the wheel pair, while the conservative
  // hip disturbance correction supplements pitch only through the common hip
  // pair. Differential yaw/roll paths remain owned by their existing loops.
  // LESO wheel compensation is useful while translating, but the measured
  // standstill trace showed that a residual correction sustained the forward
  // limit cycle.  Fade it with the same slewed speed reference so it reaches
  // exactly zero at idle without a release-step torque.
  const float wheel_compensation_blend = ClampRange(
      fabsf(normal_speed_ref_) / LESO_WHEEL_FULL_COMPENSATION_SPEED, 0.0f,
      1.0f);
  const float wheel_correction_target = ClampAbs(
      wheel_compensation_blend * leso_wheel_compensation_gain_ *
          leso_wheel_disturbance_filtered_,
      LESO_WHEEL_COMMON_LIMIT);
  const float hip_correction_target = ClampAbs(
      leso_hip_compensation_gain_ * leso_hip_disturbance_filtered_,
      LESO_HIP_COMMON_LIMIT);
  leso_wheel_correction_ = SlewTowards(
      leso_wheel_correction_, wheel_correction_target,
      LESO_WHEEL_CORRECTION_SLEW_PER_SECOND * controller_dt_);
  leso_hip_correction_ = SlewTowards(
      leso_hip_correction_, hip_correction_target,
      LESO_HIP_CORRECTION_SLEW_PER_SECOND * controller_dt_);
  const float pitch_leso_correction =
#if PITCH_LESO_COMPENSATION_ENABLE
      UpdatePitchLeso(true, INS.Pitch, INS.Gyro[1]);
#else
      UpdatePitchLeso(false, INS.Pitch, INS.Gyro[1]);
#endif

  // Restore the established post-controller wheel path. Both observers add
  // only equal left/right common corrections; the proven pivot allocator above
  // remains responsible for common/differential authority before this point.
  l_wheel_T_ = ClampAbs(l_wheel_T_ - leso_wheel_correction_ +
                            pitch_leso_correction,
                        kNormalWheelTorqueLimit);
  r_wheel_T_ = ClampAbs(r_wheel_T_ - leso_wheel_correction_ +
                            pitch_leso_correction,
                        kNormalWheelTorqueLimit);
  pitch_leso_applied_common_torque_ = 0.5f * (l_wheel_T_ + r_wheel_T_);
  left_leg_T_ = ClampAbs(left_leg_T_ - leso_hip_correction_, 40.0f);
  right_leg_T_ = ClampAbs(right_leg_T_ - leso_hip_correction_, 40.0f);

  // Feed the observer the logical, saturated virtual inputs actually handed
  // to the wheel plant and VMC. Hardware-specific left/right sign inversions
  // are applied later in SetMotorTor() and are not part of the model input.
  const float applied_input[LESO_INPUT_DIM] = {
      l_wheel_T_, r_wheel_T_, left_leg_T_, right_leg_T_};
  if (!leso_.Step(measurement, applied_input, left_leg_.GetLegLen(),
                  right_leg_.GetLegLen())) {
    ResetLeso();
  }
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
  // Translation and yaw are two simultaneous NORMAL inputs; zero translation
  // plus non-zero yaw is not a separate chassis mode.
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
    float yaw_step = kNormalPivotYawRateAccel * 0.001f;
    if (controller_dt_ > 0.0f && controller_dt_ < 0.02f) {
      yaw_step = kNormalPivotYawRateAccel * controller_dt_;
    }
    // Apply the same slew on entry, reversal and release.  In particular, do
    // not overwrite this reference with zero on the stick-release edge; the
    // unified wheel yaw loop must remain active while it brakes residual yaw.
    normal_pivot_yaw_rate_ref_ =
        SlewTowards(normal_pivot_yaw_rate_ref_, yaw_desired, yaw_step);
    target_w_rotation_ = normal_pivot_yaw_rate_ref_;
  } else {
    normal_pivot_yaw_rate_ref_ = 0.0f;
    target_w_rotation_ = yaw_desired;
  }
  // 速度目标在状态层统一管理，这里只负责写入当前期望值。

  if (fabsf(target_speed_) <= kTranslationCommandDeadband)
    target_speed_ = 0.0f;

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
    if (yaw_command_active_ &&
        fabsf(normal_pivot_yaw_rate_ref_) <= kYawCommandDeadband) {
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
