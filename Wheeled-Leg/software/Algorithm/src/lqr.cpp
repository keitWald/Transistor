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
  pitch_target_ = 0.0f;
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
  error[5] = -w_theta_l_;
  error[6] = -0.0f - theta_r_;
  error[7] = -w_theta_r_;
  error[8] = (pitch_target_ - body_) * pitch_gain_scale_;
  error[9] = -w_body_ * pitch_gain_scale_;
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
  for (uint8_t i = 0; i < 4; ++i) {
    T_[i] = T_K_[i][0] * error[0] + T_K_[i][1] * error[1] +
            T_K_[i][2] * error[2] + T_K_[i][3] * error[3] +
            T_K_[i][4] * error[4] + T_K_[i][5] * error[5] +
            T_K_[i][6] * error[6] + T_K_[i][7] * error[7] +
            T_K_[i][8] * error[8] + T_K_[i][9] * error[9];
  }
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
