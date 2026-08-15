#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "DM_8009P.h"
#include "GM6020.h"
#include "M3508.h"
#include "M6020_Motor.h"
#include "kalman_filter.h"
#include "lqr.h"
#include "motor_pid.h"
#include "pid.h"
#include "tpid.h"
#include "vmc.h"
#include <stdint.h>

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180
#define VEL_PROCESS_NOISE 10
#define VEL_MEASURE_NOISE 1000
#define ACC_PROCESS_NOISE 10
#define ACC_MEASURE_NOISE 10000
#define MAX_LEG_LENGTH 0.43f
#define MIN_LEG_LENGTH 0.23f
#define Leg_Controller_LQR_FeedForward 5.4f
#define Leg_Controller_LQR_K1 31.6228f
#define Leg_Controller_LQR_K2 1.2142f
#define LEG_FF_BODY_MASS 10.689f
#define LEG_FF_LEG_MASS 1.067f
#define LEG_FF_LEG_MASS_RATIO 1.0f
#define LEG_FF_WHEEL_TRACK 0.4f
#define kNormalPitchZeroOffset 0.050f
#define kNormalRollZeroOffset 0.006f
// --- NORMAL状态roll补偿 ---
#define kNormalRollKp 70.0f
#define kNormalRollKi 90.0f
#define kNormalRollKd 8.0f
#define kNormalRollOutMax 40.0f
#define kNormalRollIntegralMax 30.0f
#define kNormalRollForceLpfAlpha 0.06f
#define kRollDeadBand 0.003f
#define kRecoverRollKp 220.0f
#define kRecoverRollKd 24.0f
#define kRollLenRateGain 0.025f
#define kRollLenLpfAlpha 0.08f
#define kRollLenGain 0.2f // roll角 → 腿长差增益 (≤ wheel_track/2)
#define kRollLenMax 0.08f // roll腿长补偿限幅
#define kTranslationCommandDeadband 0.08f // m/s, after the SBUS ramp
#define kYawCommandDeadband 0.02f         // rad/s, after the SBUS ramp
// The wheel-torque path is recomposed into common (pitch + axle-centre speed)
// and differential (yaw) coordinates before motor commands are formed.
// The differential channel is a conventional positive plant: error -> same-sign
// torque.  The common channel is non-minimum-phase at standstill: a backward
// centre-speed error must command negative wheel torque so the body pitches
// into the motion; the previous positive sign produced a -1 m/s limit cycle.
// Keep the common and differential signs explicit because their output
// synthesis differs.
#define kNormalCenterSpeedTorqueSign (-1.0f)
#define kNormalDiffSpeedTorqueSign 1.0f
// The axle-centre channel is non-minimum-phase in every NORMAL operating
// condition.  A pivot must use the same feedback direction as normal idle;
// the opposite (+1) sign produces the measured near-straight -1 m/s retreat.
#define kPivotCenterSpeedTorqueSign (-1.0f)
#define kNormalYawRateScale 1.0f
// Limit only a zero-translation NORMAL yaw request.  SBUS shaping is shared by
// all robot states, so this extra slew belongs in the NORMAL pivot path.
#define kNormalPivotYawRateAccel 4.0f // rad/s^2
#define kPivotWheelSpeedKp 3.0f
// Outer yaw-rate loop correction in metres per second per radian per second.
// It compensates effective wheel-track and tyre-slip error before the inner
// differential wheel-speed loop, so measured yaw rate follows the RC command
// instead of an open-loop geometric estimate.
#define kPivotYawRateToDiffSpeedKp 0.10f
// NORMAL yaw is deliberately P-only.  Neither heading error nor differential
// wheel-speed error may accumulate after the stick returns to centre.
#define kPivotYawTorqueMax 6.0f
#define kPivotYawTorqueLpfAlpha 0.05f
#define kPivotPitchKi 8.0f
#define kPivotPitchIntegralMax 1.5f
#define kNormalWheelSpeedLpfAlpha 0.08f
#define kPivotLegSyncKp 1000.0f
#define kPivotLegSyncKi 400.0f
#define kPivotLegSyncKd 35.0f
#define kPivotLegSyncForceMax 80.0f
#define kPivotLegSyncIntegralMax 40.0f
#define kPivotLegSyncLpfAlpha 0.18f
#define kPivotRollYawRateFf 10.0f
#define kPivotRollYawForceMax 12.0f
#define kNormalPivotRollKp 100.0f
#define kNormalPivotRollKi 260.0f
#define kNormalPivotRollKd 14.0f
#define kNormalPivotRollForceMax 100.0f
#define kNormalPivotRollIntegralMax 80.0f
#define kNormalPivotRollForceLpfAlpha 0.10f
#define kPivotRollFullLegDiff 0.003f
#define kPivotRollCutoffLegDiff 0.010f
#define kNormalSpeedBias 0.125f
#define kNormalZeroInputSpeedBias 0.020f
#define kNormalSpeedBiasRampSpeed 0.25f
#define kNormalSpeedRefAccel 0.80f
#define kNormalSpeedRefDecel 1.20f
#define kNormalZeroSpeedKp 4.0f
// NORMAL LQR retains its ordinary speed and pitch states during a pivot.  A
// A firm midpoint P term stops common wheel speed before the trim can react;
// the pivot-only pitch gain and common-first limiter retain balance authority.
#define kPivotCenterSpeedKp 3.0f
#define kPivotCenterSpeedKi 8.0f
#define kNormalPivotWheelSpeedLpfAlpha 0.10f
#define kPivotCenterIntegratorDeadband 0.012f
#define kPivotCenterZeroCrossRetention 0.20f
// The previous slow trim left a repeatable negative I2 residual at high yaw
// rate. Allow more common-channel correction while the final 10 N.m wheel limit
// and common-first saturation still protect balance.
#define kPivotCenterIntegralMax 4.0f
#define kPivotCenterSpeedTorqueMax 8.0f
// Keep the pivot centre-speed loop bounded when pitch is displaced.  Full
// authority couples into the pitch mode and causes a rapid I2 limit cycle.
#define kPivotCenterLoopMinScale 0.30f
// NORMAL pivot positive wheel-speed bias curve.  With normalized yaw command
// x = abs(target_yaw_rate) / kNormalYawRateScale:
//   normalized_bias = A*x^3 + B*x^2 + C*x + D
// Keep the axle-midpoint target at zero.  Pivot correction must cancel the
// common drag torque itself, not command a non-zero I2 equilibrium.
#define kPivotYawBiasCubicA 0.00f
#define kPivotYawBiasCubicB 0.00f
#define kPivotYawBiasCubicC 0.00f
#define kPivotYawBiasCubicD 0.00f
// The common positive bias dominates the differential target at high stick
// input unless differential authority grows faster.  A quadratic schedule
// leaves the proven low-speed region nearly unchanged and restores opposite
// wheel motion in the medium/high-speed region.
#define kPivotHighSpeedDiffExtraGain 3.0f
// A common positive bias must never cancel the slower wheel's opposite-speed
// target, otherwise the chassis pivots around that stationary wheel.
#define kPivotPositiveBiasSafetyMaxDiffRatio 0.20f
#define kPivotSelectedWheelSpeedKp 4.0f
// The individual left/right pivot speed correction uses the same measured
// body-coordinate direction as the common and differential loops above.
#define kPivotSelectedWheelSpeedTorqueSign (1.0f)
#define kPivotSelectedWheelTorqueBaseMax 6.0f
#define kPivotSelectedWheelTorqueExtraMax 6.0f
// Decoupled pivot wheel-speed gains.  The LQR speed error is zeroed during
// yaw, so the common term only has to hold I2 at zero, not fight the LQR.
#define kPivotCommonSpeedKp 5.0f
#define kPivotDiffSpeedKp 3.0f
// Separate torque ceilings so a large common correction cannot starve the
// differential correction on either wheel.
#define kPivotCommonTorqueMax 6.0f
#define kPivotDiffTorqueMax 6.0f
// At a centred stick the common wheel coordinate is non-minimum-phase: the
// fitted LQR speed term appears anti-damping, so the zero-speed P loop uses
// the negative sign to pitch the body into the motion.  The small trim
// integral is integrated with the opposite sign in Chassis.cpp so it cancels
// the steady yaw-induced common offset instead of winding into a sawtooth.
#define kNormalZeroSpeedKi 0.5f
#define kNormalZeroSpeedIntegralMax 1.0f
#define kNormalZeroSpeedTorqueMax 3.5f
#define kNormalCenterLoopPitchFade (8.0f * DEGREE_2_RAD)
#define kNormalCenterLoopMinScale 0.50f
#define kNormalWheelTorqueLimit 10.0f
#define kNormalPivotCommonTorqueLimit 8.0f
#define kNormalDrivePitchGainScale 1.0f
#define kNormalPivotPitchGainScale 2.0f
// A small low-speed pitch support cancels the pivot drag seen on I2.  Cap it
// before the high-speed region so it cannot become a nose-up command.
#define kPivotPitchFf 0.05f
#define kPivotPitchFfMax (1.0f * DEGREE_2_RAD)
#define kPivotPitchFfSlew 0.12f // rad/s
// At zero operator input the two hip mechanisms must not receive a large
// opposing LQR torque.  Roll is controlled by differential axial support.
#define kNormalIdleLegDiffTorqueMax 3.0f
#define kNormalLegSyncKp 160.0f
#define kNormalLegSyncKd 8.0f
#define kNormalLegSyncForceMax 12.0f
#define kPivotStartCenterSpeed 0.04f
#define kPivotFullCenterSpeed 0.10f
// During a pivot, constrain the leg angle relative to the chassis rather than
// relative to the world vertical.  A world-vertical target makes both legs
// sweep rearward as soon as body pitch changes and feeds that reaction back
// into the common wheel channel.
#define kNormalPivotLegAngleKp 65.0f
#define kNormalPivotLegAngleKd 7.0f
#define kNormalPivotLegAngleTorqueMax 12.0f
#define kNormalPivotHipPitchKp 35.0f
#define kNormalPivotHipPitchKi 60.0f
#define kNormalPivotHipPitchKd 4.0f
#define kNormalPivotHipPitchIntegralMax 10.0f
#define kNormalPivotHipPitchTorqueMax 16.0f
// The hip feedforward is disabled: its quadratic high-speed component was the
// source of the continuing nose-up bias.  Wheel/LQR pitch support remains.
#define kPivotLegPitchFf 0.0f
#define kPivotLegPitchFf2 0.0f
// Preserve full yaw performance when the chassis is balanced.  Only shed yaw
// torque while pitch/leg geometry is outside the controllable region.
#define kPivotYawFullPitch (4.0f * DEGREE_2_RAD)
#define kPivotYawCutoffPitch (14.0f * DEGREE_2_RAD)
#define kPivotYawFullLegAngle (5.0f * DEGREE_2_RAD)
#define kPivotYawCutoffLegAngle (18.0f * DEGREE_2_RAD)
#define kNormalFallPitch (60.0f * DEGREE_2_RAD)
#define kNormalFallRoll (60.0f * DEGREE_2_RAD)
#define kNormalFallLegAngle (35.0f * DEGREE_2_RAD)
#define kNormalHandoffLegGraceTicks 1500U
#define recover_final_target_len 0.25f
#define recover_center_target_len 0.23f
#define recover_tmp_target_len 0.35f
#define recover_target_phi0 (160.0f * DEGREE_2_RAD)
#define theta_max (30.0f * DEGREE_2_RAD)
#define pitch_max (45.0f * DEGREE_2_RAD)
#define roll_max (45.0f * DEGREE_2_RAD)
#define leg_angle_err_max (20.0f * DEGREE_2_RAD)
#define leg_angle_accel_gain 1.0f
#define k_pitch_ok_rad (5.0f * DEGREE_2_RAD)
#define k_pitch_ok_hold 500
#define k_theta_ok_rad (10.0f * DEGREE_2_RAD)
#define k_yaw_ok_rad (5.0f * DEGREE_2_RAD)
#define k_yaw_ok_hold 1000
#define k_recover_timeout 25000
#define k_recover_sbus_loss_grace_ticks 400
#define k_shoutui_length_th 0.02f
#define k_shoutui_angle_th 0.18f
#define k_shoutui_speed_th 0.04f
#define k_shoutui_ok_hold 200
#define k_phi0_ok (20.0f * DEGREE_2_RAD)
#define k_phi0speed_ok 0.05f
#define k_recover_phi0_speed_max (60.0f * DEGREE_2_RAD)
#define k_recover_align_angle_th (10.0f * DEGREE_2_RAD)
#define k_recover_align_hold 100
#define angle_ready_th 0.4f
#define k_fall_confirm_ticks 350
#define k_recover_settle_ticks 200
#define k_recover_side_exit_hold 200
#define k_recover_center_hold 200
#define k_recover_capture_handoff_hold 20
#define k_recover_capture_handoff_pitch (12.0f * DEGREE_2_RAD)
#define k_recover_capture_handoff_speed 0.22f
#define k_recover_capture_handoff_phi_err (45.0f * DEGREE_2_RAD)
#define k_recover_balance_wheel_ramp_ticks 1200.0f
#define k_recover_balance_wheel_torque_max 1.2f
#define k_recover_balance_speed_soft_start 0.02f
#define k_recover_balance_speed_limit 0.12f
#define k_recover_balance_speed_damping 7.0f
#define k_recover_balance_reverse_alpha 0.18f
#define k_recover_balance_capture_len 0.23f
#define k_recover_balance_len_kp 120.0f
#define k_recover_balance_len_kp_final 360.0f
#define k_recover_balance_len_kd 28.0f
#define k_recover_balance_force_min (-35.0f)
#define k_recover_balance_force_max 70.0f
#define k_recover_balance_force_max_final 95.0f
#define k_recover_balance_support_ramp_delay 200.0f
#define k_recover_balance_support_ramp_ticks 1500.0f
#define k_recover_balance_complete_len_err 0.02f
#define k_recover_balance_complete_len_speed 0.08f
#define k_recover_normal_handoff_min_len 0.18f
#define k_recover_manual_handoff_hold 300U
#define k_normal_handoff_len_slew_per_tick 0.00005f
#define k_recover_joint_lift_full_len 0.18f
#define k_recover_joint_lift_release_len 0.21f
#define k_recover_joint_lift_torque_max 18.0f
#define k_recover_balance_yaw_delay_ticks 300
#define k_recover_balance_yaw_ramp_ticks 500.0f
#define k_recover_balance_yaw_torque_max 0.35f
#define k_recover_side_projection_th 0.55f
#define k_recover_side_exit_projection 0.35f
#define k_recover_side_long_len 0.38f
#define k_recover_sync_leg_len 0.24f
#define k_recover_sync_phi_kp 60.0f
#define k_recover_sync_phi_kd 10.0f
#define k_recover_sync_torque_max 30.0f
#define k_recover_sync_pair_err (8.0f * DEGREE_2_RAD)
#define k_recover_sync_speed_ok (25.0f * DEGREE_2_RAD)
#define k_recover_sync_hold 80
#define k_recover_sweep_leg_len 0.36f
#define k_recover_sweep_speed_per_tick (20.0f * DEGREE_2_RAD * 0.001f)
#define k_recover_sweep_tracking_slow (8.0f * DEGREE_2_RAD)
#define k_recover_sweep_tracking_pause (18.0f * DEGREE_2_RAD)
#define k_recover_sweep_phi_kp 60.0f
#define k_recover_sweep_phi_kd 10.0f
#define k_recover_sweep_sync_kp 25.0f
#define k_recover_sweep_sync_kd 4.0f
#define k_recover_sweep_torque_max 30.0f
#define k_recover_sweep_torque_slew_per_tick 0.08f
#define k_recover_pose_capture_rate (15.0f * DEGREE_2_RAD)
#define k_recover_pose_capture_hold 300
#define k_recover_center_entry_len_max 0.20f
#define k_recover_center_fallback_projection 0.75f
#define k_recover_center_fallback_angle (30.0f * DEGREE_2_RAD)
#define k_recover_center_fallback_hold 200
#define k_recover_pose_len_kp 140.0f
#define k_recover_pose_retract_len_kp 300.0f
#define k_recover_pose_len_kd 30.0f
#define k_recover_pose_force_min (-55.0f)
#define k_recover_pose_force_max 70.0f
#define k_recover_upright_cos 0.9063078f // cos(25 deg)
#define k_recover_body_rate_ok (20.0f * DEGREE_2_RAD)
#define k_recover_center_len_kp 100.0f
#define k_recover_center_len_kd 28.0f
#define k_recover_center_force_max 60.0f
#define k_recover_center_retract_len_kp 220.0f
#define k_recover_center_retract_force_min (-45.0f)
#define k_recover_center_retract_threshold 0.02f
#define k_recover_center_retract_ff_scale 0.20f
#define k_recover_center_overrun_angle (25.0f * DEGREE_2_RAD)
#define k_recover_center_overrun_retract_kp 320.0f
#define k_recover_center_overrun_force_min (-65.0f)
#define k_recover_center_overrun_phi_kp 60.0f
#define k_recover_center_overrun_phi_kd 8.0f
#define k_recover_center_overrun_torque_max 45.0f
#define k_recover_center_len_slew_per_tick 0.00005f
#define k_recover_balance_len_slew_per_tick 0.00005f
#define k_recover_capture_leg_boost_force 10.0f
#define k_recover_capture_leg_force_max 80.0f
#define k_recover_capture_leg_len_kd 20.0f
#define k_recover_capture_leg_boost_err_range 0.03f
#define k_recover_capture_leg_ready_err 0.015f
#define k_recover_capture_leg_ready_speed 0.12f
#define k_recover_capture_prep_len 0.175f
#define k_recover_capture_stand_len 0.21f
#define k_recover_capture_stand_slew_per_tick 0.00015f
#define k_recover_capture_stand_boost_force 15.0f
#define k_recover_capture_stand_force_max 85.0f
#define k_recover_capture_stand_kd 24.0f
#define k_recover_center_phi_kp 45.0f
#define k_recover_center_phi_kd 4.0f
#define k_recover_center_torque_max 36.0f
#define k_recover_center_pitch_comp_gain 0.35f
#define k_recover_center_pitch_comp_max (6.0f * DEGREE_2_RAD)
#define k_recover_center_gate_up_projection 0.80f
#define k_recover_center_wheel_torque_max 1.5f
#define k_recover_center_wheel_ramp_ticks 300.0f
#define k_recover_center_lqr_theta_scale 0.20f
#define k_recover_wheel_ready_len_min 0.125f
#define k_recover_wheel_ready_phi_err (25.0f * DEGREE_2_RAD)
#define k_recover_wheel_ready_len_speed 0.15f
#define k_recover_wheel_ready_phi_speed (45.0f * DEGREE_2_RAD)
#define k_recover_wheel_ready_hold 40
#define k_recover_capture_handoff_len_min 0.15f
#define k_recover_wheel_latched_up_projection 0.75f
#define k_recover_wheel_latched_phi_err (35.0f * DEGREE_2_RAD)
#define k_recover_wheel_latched_len_min 0.12f
#define k_recover_wheel_latched_len_max MAX_LEG_LENGTH
#define k_recover_wheel_speed_soft_start 0.10f
#define k_recover_wheel_speed_limit 0.25f
#define k_recover_wheel_speed_damping 4.0f
#define k_recover_wheel_torque_lpf_alpha 0.015f
#define k_recover_wheel_breakaway_torque 0.20f
#define k_recover_wheel_breakaway_fade_start 0.20f
#define k_recover_wheel_breakaway_pitch_off (5.0f * DEGREE_2_RAD)
#define k_recover_wheel_breakaway_pitch_full (12.0f * DEGREE_2_RAD)
#define k_recover_wheel_breakaway_delay_ticks 160
#define k_recover_wheel_breakaway_duration_ticks 60
#define k_recover_wheel_torque_reverse_alpha 0.06f
#define k_recover_roll_force_max 30.0f
#define k_recover_roll_force_lpf_alpha 0.05f
#define k_recover_emergency_speed 0.30f
#define k_recover_emergency_brake_gain 8.0f
#define k_recover_emergency_brake_torque 3.0f
// --- 参考山海机甲开源：倒地自起控制常量 ---
#define k_recover_retracted_len 0.10f // 收腿目标长度 L0
#define k_recover_phi0_target                                                  \
  (1.4f * PI) // 收腿目标摆杆角度 phi0 = 252°（腿撑地位置）
#define k_recover_shoutui_angle_joint_th 0.18f           // 关节角度到位阈值
#define k_recover_shoutui_speed_joint_th 0.04f           // 关节速度停止阈值
#define k_recover_shoutui_joint_hold 200                 // 收腿完成保持计数
#define k_recover_pitch_balance_th (5.0f * DEGREE_2_RAD) // 平衡判定俯仰角阈值
#define k_recover_pitch_balance_hold 500                 // 平衡保持计数
#define k_recover_pitch_yaw_hold 500 // Yaw对齐保持计数(与节同步)
// 参考山海机甲开源：起身 Yaw 对齐控制（Locomotion_Controller_Yaw_Control）
#define k_recover_yaw_k1 4.4721f   // 偏航角误差 -> 角速度指令
#define k_recover_yaw_k2 6.3246f   // 角速度误差 -> 差分轮矩
#define k_recover_yaw_w_limit 2.5f // 角速度指令限幅 (w_Limit)
// --- 倒地/收腿阻尼常量 ---
// 急停行为：全停零力矩（所有电机停止），不再分级阻尼/支撑
#define kWarmingWheelDamping 1.0f // 收腿阶段轮组阻尼系数
// 调试模式
#define CHASSIS_JOINT_DEBUG_ENABLE 0
#define CHASSIS_JOINT_DEBUG_PHI0 1.57f
#define CHASSIS_JOINT_DEBUG_L0 0.20f
#define CHASSIS_JOINT_DEBUG_KP 25.0f
#define CHASSIS_JOINT_DEBUG_KD 3.0f
#define CHASSIS_TEACH_ENABLE 0
// 离地检测
#define OFF_GROUND_DETECT_ENABLE 1       // 1: 开启检测
#define OFF_GROUND_THRESHOLD 30.0f       // 法向力阈值
#define OFF_GROUND_LEG_LENGTH 0.4f       // 离地后的目标腿长
#define OFF_GROUND_ENTER_THRESHOLD 10.0f // both legs below this to enter
#define OFF_GROUND_EXIT_THRESHOLD 30.0f  // either leg above this to exit
#define OFF_GROUND_ENTER_TICKS 30U       // 30 ms confirmation at 1 kHz
#define OFF_GROUND_EXIT_TICKS 10U        // fast landing confirmation
// 跳跃阶段：轮速锁零、腿摆角锁定和腿长到位判定
#define JUMP_WHEEL_SPEED_KP 1.10f         // N.m/(rad/s), jump-only motor speed loop P
#define JUMP_WHEEL_SPEED_KI 35.0f         // N.m/(rad), jump-only motor speed loop I
#define JUMP_WHEEL_TORQUE_MAX 4.8f        // N.m, below the 3508+C620 peak limit
#define JUMP_WHEEL_INTEGRAL_MAX 2.5f      // N.m
#define JUMP_ASCEND_WHEEL_SPEED_CAPTURE_SCALE 1.0f
#define JUMP_WHEEL_PITCH_REF_KP 35.0f     // (rad/s)/rad
#define JUMP_WHEEL_PITCH_REF_KD 4.0f      // (rad/s)/(rad/s)
#define JUMP_WHEEL_PITCH_REF_MAX 30.0f    // max common speed-reference trim
#define JUMP_RETRACT_WHEEL_PITCH_REF_KP 55.0f
#define JUMP_RETRACT_WHEEL_PITCH_REF_KD 7.0f
#define JUMP_RETRACT_WHEEL_PITCH_REF_MAX 40.0f
#define JUMP_LAND_FLIGHT_DECEL_START_TICKS 20U
#define JUMP_LAND_FLIGHT_SPEED_SCALE 0.75f
#define JUMP_LAND_FLIGHT_SPEED_SLEW 0.003f
#define JUMP_LAND_WHEEL_TORQUE_MAX 2.5f   // limit landing braking reaction
#define JUMP_LAND_RAW_CONTACT_SPEED_SCALE 0.65f
#define JUMP_LAND_RAW_CONTACT_SPEED_SLEW 0.010f
#define JUMP_LAND_RAW_CONTACT_TORQUE_MAX 1.50f
#define JUMP_LAND_SINGLE_CONTACT_SPEED_SCALE 0.55f
#define JUMP_LAND_SINGLE_SPEED_SLEW 0.012f
#define JUMP_LAND_BOTH_SPEED_SLEW 0.0125f
#define JUMP_LAND_CONTACT_CONFIRM_TICKS 10U
#define JUMP_LAND_CONTACT_RELEASE_TICKS 20U
#define JUMP_LAND_BRAKE_REF_SLEW 0.70f
#define JUMP_LAND_BRAKE_SPEED_KP 0.22f
#define JUMP_LAND_BRAKE_TORQUE_MAX 1.40f
#define JUMP_LAND_BRAKE_PITCH_TRIM_SCALE 0.35f
#define JUMP_LAND_BRAKE_ZERO_CROSS_SPEED 2.0f
#define JUMP_LAND_EXIT_CHASSIS_SPEED 10.0f
#define JUMP_PHI0_TARGET (0.5f * PI)
#define JUMP_PHI0_KP 60.0f
#define JUMP_PHI0_KD 6.0f
#define JUMP_PHI0_TORQUE_MAX 18.0f
#define JUMP_LAND_PHI0_KP 40.0f
#define JUMP_LAND_PHI0_KD 10.0f
#define JUMP_LAND_PHI0_TORQUE_MAX 12.0f
#define JUMP_LAND_CONTACT_PHI0_KP 28.0f
#define JUMP_LAND_CONTACT_PHI0_KD 14.0f
#define JUMP_LAND_CONTACT_PHI0_TORQUE_MAX 8.0f
#define JUMP_LAND_PHI0_REF_SLEW_PER_TICK 0.0025f
#define JUMP_PITCH_KP 100.0f
#define JUMP_PITCH_KI 70.0f
#define JUMP_PITCH_KD 14.0f
#define JUMP_AIR_PITCH_KD 22.0f
#define JUMP_RETRACT_PITCH_KP 200.0f
#define JUMP_RETRACT_PITCH_KD 42.0f
#define JUMP_PITCH_INTEGRAL_MAX 8.0f
#define JUMP_PITCH_TORQUE_MAX 28.0f
#define JUMP_ROLL_KP 180.0f
#define JUMP_ROLL_KI 120.0f
#define JUMP_ROLL_KD 20.0f
#define JUMP_ROLL_INTEGRAL_MAX 20.0f
#define JUMP_ROLL_FORCE_MAX 45.0f
#define JUMP_EXTEND_LENGTH_TOLERANCE 0.0002f
#define JUMP_COMPRESS_LENGTH 0.20f
#define JUMP_RETRACT_LENGTH 0.20f
#define JUMP_RETRACT_LENGTH_TOLERANCE 0.010f
#define JUMP_LENGTH_READY_TICKS 4U
#define JUMP_RETRACT_AT_LENGTH_HOLD_TICKS 40U // hold 0.20 m after both legs arrive
#define JUMP_LAND_PREP_LENGTH 0.32f
#define JUMP_LAND_PREP_SLEW_PER_TICK 0.0015f
#define JUMP_LAND_PREP_KP 350.0f
#define JUMP_LAND_PREP_KD 28.0f
#define JUMP_LAND_PREP_FORCE_MIN (-45.0f)
#define JUMP_LAND_PREP_FORCE_MAX 100.0f
#define JUMP_LANDING_CONFIRM_TICKS 80U
#define JUMP_NORMAL_HANDOFF_TICKS 250U
#define JUMP_NORMAL_WHEEL_RAMP_TICKS 100U
#define JUMP_NORMAL_WHEEL_MIN_SCALE 0.45f
#define DM_MANUAL_ENABLE_WAIT_TICKS 250U
// 状态值约定（VOFA 上位机显示）：1=normal，2=recover
enum RobotStatus {
  STATE_NORMAL = 1,
  STATE_RECOVERING = 2,
  STATE_JUMPING = 3,
  STATE_JOINT_DEBUG = 4,
  STATE_ESTOP = 5
};
enum JumpSubStatus {
  JUMP_NONE = 0,
  JUMP_COMPRESS,
  JUMP_ASCEND,
  JUMP_RETRACT,
  JUMP_LAND_PREP
};
enum RecoverSubStatus {
  RECOVER_SETTLE = 0,
  RECOVER_ROLL_TO_SAGITTAL,
  RECOVER_SWEEP,
  RECOVER_CENTER_LEGS,
  RECOVER_BALANCE
};
typedef struct Speed_ToCloud {
  float vx;
  float vy;
  float wz;
} Speed_ToCloud;
typedef struct Speed_ToChassis {
  float vx;
  float vy;
  float wz;
} Speed_ToChassis;
#ifdef __cplusplus
class balance_Chassis {
public:
  Class_Motor_DM_8009P lf_joint_, lb_joint_, rf_joint_, rb_joint_;
  Class_Motor_3508 left_wheel, right_wheel;
  Vmc left_leg_, right_leg_;
  KalmanFilter_t kf, kf_l, kf_r;
  Lqr lqr_body_;
  Pid left_leg_len_, right_leg_len_, anti_crash_, roll_comp_, left_leg_phi0,
      right_leg_phi0, left_leg_phi0_speed_, right_leg_phi0_speed_;
  Speed_ToCloud speed_to_cloud;
  Speed_ToChassis speed_to_chassis;
  void LegCalc();
  void MotorInit();
  void PidInit();
  void StatusInit();
  void Rev();
  void TorCalc();    // 根据 LQR 和 VMC 计算关节力矩
  void TorControl(); // 按当前状态下发电机控制
  void LQRCalc();
  void LegLenCalc(float left_ref, float right_ref, float left_ff,
                  float right_ff);
  void SpeedCalc();
  void SynthesizeMotion();
  void SpeedEstInit();
  void SetMotorTor();
  void StopMotor();
  void SetLegLen();
  void SetFollow();
  void SetState(); // 更新状态机并刷新控制目标
  void SetSpd();   // 根据遥控输入设置速度目标
  void getangle(float angle);
  void Observe();
  void Filter(float dtheta_b_, float dphi_);
  void OffGroundDetect();
  float GetSpeed() { return vel_m; }
  float GetSpeed_filter() { return vel_; }
  float GetNormalCenterSpeed() { return normal_wheel_center_speed_; }
  float GetNormalDiffSpeed() { return normal_wheel_diff_speed_; }
  float GetNormalTargetLeftWheelSpeed() {
    return normal_target_left_wheel_speed_;
  }
  float GetNormalTargetRightWheelSpeed() {
    return normal_target_right_wheel_speed_;
  }
  float GetNormalActualLeftWheelSpeed() {
    return normal_wheel_center_speed_ - normal_wheel_diff_speed_;
  }
  float GetNormalActualRightWheelSpeed() {
    return normal_wheel_center_speed_ + normal_wheel_diff_speed_;
  }
  float GetTargetSpeed() { return target_speed_; }
  float GetLeftBodySpeed() { return left_v_body_; }
  float GetRightBodySpeed() { return right_v_body_; }
  float GetRollLengthCorrection() { return roll_len_delta_cmd_; }
  float GetNormalRollForceCmd() { return roll_force_cmd_; }
  float GetNormalLegSyncForce() { return normal_pivot_leg_sync_force_; }
  float GetNormalPivotCenterTrim() { return normal_pivot_center_trim_torque_; }
  float GetNormalPivotLeftCorr() { return normal_pivot_left_wheel_corr_; }
  float GetPositionError() { return target_dist_ - dist_; }
  float GetTargetYawRate() { return target_w_rotation_; }
  float GetHeadingError() { return target_rotation_ - rotation_; }
  float GetLeftLegTor() { return left_leg_T_; }
  float GetLeftLegForce() { return left_leg_F_; }
  float GetRightLegForce() { return right_leg_F_; }
  float GetRightLegTor() { return right_leg_T_; }
  float GetRecoverJointLiftTorque() { return recover_joint_lift_torque_; }
  float GetLeftWheelTor() { return l_wheel_T_; }
  float GetRightWheelTor() { return r_wheel_T_; }
  float GetActiveLeftLegRef() { return active_left_leg_ref_; }
  float GetActiveRightLegRef() { return active_right_leg_ref_; }
  float GetJumpLandingWorldThetaRef();
  float GetJumpCapturedWheelSpeed() {
    // The two motors use opposite positive directions. Convert their raw
    // shaft speeds to one forward-positive chassis-equivalent speed.
    return 0.5f * (jump_wheel_speed_ref_r_ - jump_wheel_speed_ref_l_);
  }
  float GetJumpActiveWheelSpeedRef() {
    return 0.5f * (jump_active_wheel_ref_r_ - jump_active_wheel_ref_l_);
  }
  int GetJumpSubStatus() { return jump_status; }
  uint32_t GetJumpLengthReadyCount() { return jump_length_ready_count_; }
  float GetRecoverLenRef() { return recover_dynamic_len_ref_; }
  float GetRecoverPhi0Ref();
  bool GetRecoverState() { return recover_state_; }
  bool GetDaoDiFlg() { return DaoDiFlg_; }
  bool GetRecoverFailed() { return recover_failed_; }
  void SetDaoDiFlg(bool val) { DaoDiFlg_ = val; }
  void SetZFlag(bool val) { Z_Flag_ = val; }
  // 顶层控制
  void Controller();
  void UpdateChassisStatus(); // 旧接口，保留兼容
  void ChassissControl();
  // 状态相关控制
  void NormalCalc();
  void RecoverCalc();
  void JumpCalc();
  void JointDebugCalc();
  bool GetOffGround() { return off_ground_; }
  bool GetJumpBothContactLatched() { return jump_both_contact_latched_; }

  int GetRobotStatus() { return robot_status; }
  int GetRecoverSubStatus() { return recover_sub_status_; }
  uint32_t GetRecoverTimer() { return recover_timer; }
  bool GetRecoverWheelReady() {
    return recover_center_wheel_count_ >= k_recover_wheel_ready_hold;
  }
  float GetRecoverWheelReadyProgress() {
    const float progress =
        (float)recover_center_wheel_count_ / (float)k_recover_wheel_ready_hold;
    return progress > 1.0f ? 1.0f : progress;
  }
  uint8_t GetEstopReason() { return estop_reason_; }
  uint8_t GetDmFaultMask() { return dm_fault_latched_mask_; }
  uint8_t GetDmOfflineMask() { return dm_offline_latched_mask_; }
  uint8_t GetDmDisabledMask() { return dm_disabled_latched_mask_; }

private:
  void ResetRecoverState();                // 重置起身状态相关标志
  void ResetJumpState();                   // 重置跳跃子状态机
  void ChangeState(RobotStatus new_state); // 统一处理状态切换动作
  void UpdateStateMachine();               // 只负责状态转移
  void UpdateCommandByState();             // 按状态刷新控制目标
  bool IsAutoFallTriggered();              // 自动倒地判定
  bool IsRecoverComplete();                // 起身完成判定
  uint8_t GetDmFaultMaskNow() const;
  uint8_t GetDmOfflineMaskNow() const;
  uint8_t GetDmDisabledMaskNow() const;
  void SetJointDebugMotor();               // joint debug 模式的输出下发
  // 根据这条腿当前是在“身前”还是“身后”，设置髋关节 PID 目标和腿长参考。
  // 长路径前半段仅做一件事：把送给 PID 的 phi0 测量值加上 2pi。
  void ConfigureRecoverLegControl(float current_phi0, bool use_long_path,
                                  bool passed_pi, Pid &phi0_pid,
                                  float &leg_len_ref);
  // 参考山海机甲：使用IK+关节角LQR的收腿控制(倒地自起)
  void RecoverLegJointLQR(Vmc &leg, float target_phi0, float target_l0,
                          float &T1, float &T2);
  // 参考山海机甲：倒地后电机的阻尼/支撑控制
  void WarmingMotorControl();
  float left_leg_F_, right_leg_F_, roll_comp;
  float l_wheel_T_, r_wheel_T_, left_leg_T_, right_leg_T_;
  float target_rotation_, target_w_rotation_, target_speed_, target_dist_, vel_,
      dist_m_, dist_, acc_, rotation_, w_rotation_, target_len_;
  float vel_m, left_v_body_, right_v_body_, left_w_wheel_, right_w_wheel_;
  float dtheta_b_filter_, dist_filter_, dphi_filter_;
  float jump_start_time_, jump_now_time_;
  uint32_t dwt_cnt_controller_;
  float joint_debug_lf_target_ = 0.0f;
  float joint_debug_lb_target_ = 0.0f;
  float joint_debug_rf_target_ = 0.0f;
  float joint_debug_rb_target_ = 0.0f;
  float joint_debug_vel_limit_ = 0.0f;
  bool jump_state_ = false;
  bool joint_debug_output_enable_ = false;
  uint8_t last_jump_flag_ = 0;
  bool recover_state_ = false;
  bool recover_align_done_ = false;
  bool recover_phi0_ref_init_ = false;
  bool recover_phi0_unwrap_init_ = false;
  bool recover_enable_pending_ = false;
  bool dm_estop_confirmed_ = false;
  uint32_t dm_enable_wait_count_ = 0;
  bool recover_long_path_l_ = false;
  bool recover_long_path_r_ = false;
  bool recover_decided_l_ = false;
  bool recover_decided_r_ = false;
  bool recover_long_cross_zero_l_ = false;
  bool recover_long_cross_zero_r_ = false;
  bool recover_long_passed_pi_l_ = false;
  bool recover_long_passed_pi_r_ = false;
  bool recover_passed_pi_l_ =
      false; // 扫腿阶段：腿已越过 90°(PI/2) 目标（测量值不再 +2PI）
  bool recover_passed_pi_r_ = false;
  // 参考山海机甲：倒地自起两阶段控制标志
  bool recover_leg_retracted_ = false; // 收腿完成标志
  bool recover_failed_ = false;
  bool recover_request_latched_ = false;
  bool normal_handoff_active_ = false;
  bool DaoDiFlg_ = true;    // 倒地标志(上电默认认为倒地)
  bool Z_Flag_ = false;     // Z键释放触发标志
  bool Last_Z_Key_ = false; // 上一次Z键状态，用于下降沿检测
  float recover_phi0_ref_l_ = 0.0f;
  float recover_phi0_ref_r_ = 0.0f;
  float recover_phi0_unwrap_l_ = 0.0f;
  float recover_phi0_unwrap_r_ = 0.0f;
  float recover_phi0_last_l_ = 0.0f;
  float recover_phi0_last_r_ = 0.0f;
  float recover_wheel_torque_lpf_l_ = 0.0f;
  float recover_wheel_torque_lpf_r_ = 0.0f;
  float recover_sweep_phi_ref_ = 0.0f;
  float recover_sweep_phi_goal_ = 0.0f;
  int32_t recover_phi0_wrap_l_ = 0;
  int32_t recover_phi0_wrap_r_ = 0;
  float controller_dt_;
  float Angle_ChassisToCloud;
  float recover_dynamic_len_ref_;
  float roll_force_cmd_ = 0.0f;
  float normal_pivot_roll_integral_ = 0.0f;
  float roll_len_delta_cmd_ = 0.0f;
  float heading_last_ = 0.0f;
  bool heading_initialized_ = false;
  bool translation_command_active_ = false;
  bool yaw_command_active_ = false;
  float normal_pivot_yaw_rate_ref_ = 0.0f;
  float normal_pivot_yaw_torque_cmd_ = 0.0f;
  float normal_pivot_center_trim_torque_ = 0.0f;
  float normal_pivot_center_prev_speed_ = 0.0f;
  float normal_pivot_left_wheel_corr_ = 0.0f;
  float normal_pivot_pitch_target_ = 0.0f;
  float normal_zero_speed_trim_torque_ = 0.0f;
  float normal_speed_ref_ = 0.0f;
  float normal_wheel_center_speed_ = 0.0f;
  float normal_wheel_diff_speed_ = 0.0f;
  float normal_target_left_wheel_speed_ = 0.0f;
  float normal_target_right_wheel_speed_ = 0.0f;
  float normal_pivot_leg_sync_force_ = 0.0f;
  float normal_pivot_leg_sync_integral_ = 0.0f;
  float normal_pivot_left_speed_integral_ = 0.0f;
  float normal_pivot_right_speed_integral_ = 0.0f;
  float normal_pivot_pitch_integral_ = 0.0f;
  uint32_t normal_handoff_leg_grace_count_ = 0;
  float recover_joint_lift_torque_ = 0.0f;
  float normal_handoff_len_l_ = 0.0f;
  float normal_handoff_len_r_ = 0.0f;
  RobotStatus robot_status;
  JumpSubStatus jump_status;
  RecoverSubStatus recover_sub_status_;
  bool off_ground_ = false;
  uint32_t off_ground_enter_count_ = 0;
  uint32_t off_ground_exit_count_ = 0;
  uint32_t recover_timer;
  uint32_t fall_detect_count_;
  uint32_t balance_count;
  uint32_t align_count;
  uint32_t recover_align_count_;
  uint32_t shoutui_count;
  uint32_t shoutui_count1;
  uint32_t jump_timer;
  uint32_t jump_length_ready_count_ = 0;
  uint32_t jump_retract_hold_count_ = 0;
  uint32_t jump_landing_ready_count_ = 0;
  float jump_wheel_integral_l_ = 0.0f;
  float jump_wheel_integral_r_ = 0.0f;
  float jump_wheel_speed_ref_l_ = 0.0f;
  float jump_wheel_speed_ref_r_ = 0.0f;
  float jump_active_wheel_ref_l_ = 0.0f;
  float jump_active_wheel_ref_r_ = 0.0f;
  float jump_landing_speed_scale_ = 1.0f;
  float jump_land_phi0_ref_l_ = 0.0f;
  float jump_land_phi0_ref_r_ = 0.0f;
  float jump_landing_brake_ref_l_ = 0.0f;
  float jump_landing_brake_ref_r_ = 0.0f;
  uint16_t jump_left_contact_count_ = 0;
  uint16_t jump_right_contact_count_ = 0;
  uint16_t jump_left_release_count_ = 0;
  uint16_t jump_right_release_count_ = 0;
  bool jump_left_contact_ = false;
  bool jump_right_contact_ = false;
  bool jump_both_contact_latched_ = false;
  bool jump_landing_zero_cross_l_ = false;
  bool jump_landing_zero_cross_r_ = false;
  float jump_pitch_integral_ = 0.0f;
  float jump_roll_integral_ = 0.0f;
  float active_left_leg_ref_ = 0.0f;
  float active_right_leg_ref_ = 0.0f;
  bool jump_liftoff_seen_ = false;
  uint32_t jump_normal_handoff_count_ = 0;
  uint32_t warming_counter_;     // 倒地阻尼计数器
  uint32_t recover_balance_cnt_; // 起身平衡检测计数器
  float recover_yaw_target_;     // 起身目标朝向（进 RECOVERING 时锁定 INS.Yaw）
  uint32_t recover_yaw_cnt_;     // 起身 Yaw 对齐保持计数（Pitch 平衡后再对齐）
  uint32_t recover_sub_timer_;
  uint32_t recover_sub_stable_count_;
  uint32_t recover_center_wheel_count_;
  uint32_t recover_capture_pulse_count_;
  float recover_capture_pitch_sign_;
  uint32_t recover_pose_lost_count_;
  uint32_t recover_sbus_lost_count_;
  uint8_t estop_reason_; // 1=SBUS, 2=remote, 3=timeout, 4=fault, 5=offline, 6=disabled
  uint8_t dm_fault_latched_mask_ = 0;
  uint8_t dm_offline_latched_mask_ = 0;
  uint8_t dm_disabled_latched_mask_ = 0;
};
extern balance_Chassis chassis;
#endif
#endif
