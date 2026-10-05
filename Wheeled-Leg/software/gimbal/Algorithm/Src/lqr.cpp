/**
 *******************************************************************************
 * @file      : lqr.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "lqr.h"
#include "Chassis.h"
#include "stdint.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// LQR gain polynomial coefficients are defined in Algorithm/src/lqr_coeffs.cpp
// and evaluated by LqrGains.
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

Lqr::Lqr() {
  dist_ = speed_ = rotation_ = w_rotation_ = 0.0f;
  theta_l_ = w_theta_l_ = theta_r_ = w_theta_r_ = 0.0f;
  body_ = w_body_ = 0.0f;
  leg_len_l_ = leg_len_r_ = 0.0f;

  target_speed_ = target_dist_ = target_rotation_ = target_w_rotation_ = 0.0f;
  speed_bias_ = 0.14f;
  pitch_gain_scale_ = 1.0f;
  body_rate_gain_scale_ = 1.0f;
  pitch_target_ = 0.0f;
  common_wheel_speed_contribution_ = 0.0f;
  common_wheel_leg_contribution_ = 0.0f;
  common_wheel_leg_angle_contribution_ = 0.0f;
  common_wheel_leg_rate_contribution_ = 0.0f;
  common_wheel_pitch_contribution_ = 0.0f;
  raw_common_wheel_torque_ = 0.0f;
  F_N_L_ = F_N_R_ = 0.0f;

  for (uint8_t i = 0; i < 4; ++i) {
    T_[i] = 0.0f;
    U_[i] = 0.0f;
    for (uint8_t j = 0; j < 10; ++j) {
      T_K_[i][j] = 0.0f;
    }
  }
  for (uint8_t i = 0; i < 10; ++i) {
    error[i] = 0.0f;
  }
}
/**
 * @brief 设置状态量误差值
 */
void Lqr::SetError() {
  const float DEAD_ZONE_BODY = 0.008f; // 约 0.5 度，根据实际情况调整
  // Use the state-error convention of the gain set used by the stable
  // Wednesday 21:18 normal-driving build.
  error[0] = target_dist_ - dist_;
  error[1] = target_speed_ + speed_bias_ - speed_;
  error[2] = target_rotation_ - rotation_;
  error[3] = target_w_rotation_ - w_rotation_;
  error[4] = -0.0f - theta_l_;
  // w_theta = phi0_rate + body_rate.  GyroY used to be multiplied by
  // DEGREE_2_RAD before entering this fitted controller.  Apply the unit
  // migration to the body-rate gain only; phi0_rate has always been rad/s and
  // must retain its original damping gain.
  const float migrated_body_rate = w_body_ * body_rate_gain_scale_;
  const float body_rate_migration_correction = w_body_ - migrated_body_rate;
  error[5] = -w_theta_l_ + body_rate_migration_correction;
  error[6] = -0.0f - theta_r_;
  error[7] = -w_theta_r_ + body_rate_migration_correction;
  error[8] = (pitch_target_ - body_) * pitch_gain_scale_;
  error[9] = -migrated_body_rate * pitch_gain_scale_;
}
/**
 * @brief 输出限幅
 */
float Lqr::LimitOutput(float _u, float _max) {
  if (_u > _max) {
    _u = _max;
  }
  if (_u < -_max) {
    _u = -_max;
  }
  return _u;
}
/**
 * @brief LQR总计算函数
 */
void Lqr::Calc() {
  SetError();
  get_K_length();

  // Decompose the raw common-wheel request by physical state group. Averaging
  // rows 0/1 removes the differential yaw channel. These diagnostics are
  // calculated before per-wheel saturation so a noisy state cannot be hidden
  // merely because the final motor command has clipped.
  common_wheel_speed_contribution_ =
      0.5f * ((T_K_[0][0] + T_K_[1][0]) * error[0] +
              (T_K_[0][1] + T_K_[1][1]) * error[1]);
  common_wheel_leg_angle_contribution_ =
      0.5f * ((T_K_[0][4] + T_K_[1][4]) * error[4] +
              (T_K_[0][6] + T_K_[1][6]) * error[6]);
  common_wheel_leg_rate_contribution_ =
      0.5f * ((T_K_[0][5] + T_K_[1][5]) * error[5] +
              (T_K_[0][7] + T_K_[1][7]) * error[7]);
  common_wheel_leg_contribution_ = common_wheel_leg_angle_contribution_ +
                                   common_wheel_leg_rate_contribution_;
  common_wheel_pitch_contribution_ =
      0.5f * ((T_K_[0][8] + T_K_[1][8]) * error[8] +
              (T_K_[0][9] + T_K_[1][9]) * error[9]);

  for (uint8_t i = 0; i < 4; ++i) {
    T_[i] = T_K_[i][0] * error[0] + T_K_[i][1] * error[1] +
            T_K_[i][2] * error[2] + T_K_[i][3] * error[3] +
            T_K_[i][4] * error[4] + T_K_[i][5] * error[5] +
            T_K_[i][6] * error[6] + T_K_[i][7] * error[7] +
            T_K_[i][8] * error[8] + T_K_[i][9] * error[9];
  }
  raw_common_wheel_torque_ = 0.5f * (T_[0] + T_[1]);
  U_[0] = LimitOutput(T_[0], 10.0f);
  U_[1] = LimitOutput(T_[1], 10.0f);
  U_[2] = LimitOutput(T_[2], 40.0f);
  U_[3] = LimitOutput(T_[3], 40.0f);
}
/**
 * @brief 由左右腿长求得K值
 */
void Lqr::get_K_length() {
  lqr_gains_calculator_.calculate(leg_len_l_, leg_len_r_, off_ground_, T_K_);
}
