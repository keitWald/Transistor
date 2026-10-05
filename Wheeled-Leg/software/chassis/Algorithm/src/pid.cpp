/**
 *******************************************************************************
 * @file      : pid.cpp
 * @brief     : PID 控制器实现文件
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. 初始版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* 头文件包含 ----------------------------------------------------------------*/
#include "pid.h"

#include "bsp_dwt.h"
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有常量 ------------------------------------------------------------------*/
/* 私有类型 ------------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 外部变量 ------------------------------------------------------------------*/
/* 私有函数声明 --------------------------------------------------------------*/

/**
 * @brief 清除 PID 运行时状态
 * @note 保留 Kp、Ki、Kd、限幅和改进项配置，只清空积分、微分历史和输入输出状态
 */
void Pid::Clear() {
  ref_ = 0.0f;
  measure_ = 0.0f;
  last_measure_ = 0.0f;
  err_ = 0.0f;
  last_err_ = 0.0f;
  p_out_ = 0.0f;
  i_out_ = 0.0f;
  d_out_ = 0.0f;
  last_d_out_ = 0.0f;
  i_term_ = 0.0f;
  last_i_term_ = 0.0f;
  output_ = 0.0f;
  last_output_ = 0.0f;
  dt = 0.0f;
  DWT_CNT = DWT->CYCCNT;
  error_handle.ERRORCount = 0;
  error_handle.ERRORType = PID_ERROR_NONE;
}

/**
 * @brief 计算当前 PID 输出
 * @return PID 输出值
 */
float Pid::Calculate() {
  if (improve_ & PID_ERROR_HANDLE) {
    ErrorHandle();
  }

  dt = DWT_GetDeltaT(&DWT_CNT);
  if (dt <= 0.0f) {
    return output_;
  }

  err_ = ref_ - measure_;

  if (fabsf(err_) > dead_band_) {
    p_out_ = kp_ * err_;
    i_term_ = ki_ * err_ * dt;
    d_out_ = kd_ * (err_ - last_err_) / dt;

    if (improve_ & PID_TRAPEZOID_INTEGRAL) {
      TrapezoidIntergral();
    }
    if (improve_ & PID_CHANGING_INTEGRATION_RATE) {
      ChangingIntegratioRate();
    }
    if (improve_ & PID_DERIVATIVE_ON_MEASUREMENT) {
      DerivativeOnMeasurement();
    }
    if (improve_ & PID_DERIVATIVE_FILTER) {
      DerivativeFilter();
    }
    if (improve_ & PID_INTEGRAL_LIMIT) {
      IntegralLimit();
    }

    i_out_ += i_term_;
    output_ = p_out_ + i_out_ + d_out_;

    if (improve_ & PID_OUTPUT_FILTER) {
      OutputFilter();
    }

    OutputLimit();
  } else {
    output_ = 0.0f;
    i_term_ = 0.0f;
  }

  last_measure_ = measure_;
  last_err_ = err_;
  last_output_ = output_;
  last_d_out_ = d_out_;
  last_i_term_ = i_term_;

  return output_;
}

/**
 * @brief 梯形积分
 * @note 使用当前误差和上一拍误差的平均值更新积分项
 */
void Pid::TrapezoidIntergral() {
  i_term_ = ki_ * ((err_ + last_err_) / 2.0f) * dt;
}

/**
 * @brief 变速率积分
 * @note 当误差较大时减弱积分作用，降低积分饱和风险
 */
void Pid::ChangingIntegratioRate() {
  if (err_ * i_out_ > 0) {
    if (fabsf(err_) <= coef_b_) {
      return;
    }
    if (fabsf(err_) <= (coef_a_ + coef_b_)) {
      i_term_ *= (coef_a_ - fabsf(err_) + coef_b_) / coef_a_;
    } else {
      i_term_ = 0;
    }
  }
}

/**
 * @brief 积分限幅
 * @note 同时限制积分项本身和积分引入后的总输出趋势
 */
void Pid::IntegralLimit() {
  float temp_i_out = i_out_ + i_term_;
  float temp_output = p_out_ + temp_i_out + d_out_;
  if (fabsf(temp_output) > max_out_) {
    if (err_ * i_out_ > 0) {
      i_term_ = 0;
    }
  }

  if (temp_i_out > integral_limit_) {
    i_term_ = 0;
    i_out_ = integral_limit_;
  }
  if (temp_i_out < -integral_limit_) {
    i_term_ = 0;
    i_out_ = -integral_limit_;
  }
}

/**
 * @brief 测量微分
 * @note 使用测量值变化率代替误差变化率计算 D 项，可减小设定值突变带来的导数冲击
 */
void Pid::DerivativeOnMeasurement() {
  d_out_ = kd_ * (last_measure_ - measure_) / dt;
}

/**
 * @brief D 项低通滤波
 * @note 对 D 项做一阶低通，减弱微分放大噪声的问题
 */
void Pid::DerivativeFilter() {
  d_out_ = d_out_ * dt / (derivative_lpf_rc_ + dt) +
           last_d_out_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt); // 一阶低通滤波
}

/**
 * @brief 输出滤波
 * @note 对最终输出做一阶低通，减小输出抖动
 */
void Pid::OutputFilter() {
  output_ = output_ * dt / (output_lpf_rc_ + dt) +
            last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt);
}

/**
 * @brief 输出限幅
 */
void Pid::OutputLimit() {
  if (output_ > max_out_) {
    output_ = max_out_;
  }
  if (output_ < -max_out_) {
    output_ = -max_out_;
  }
}

/**
 * @brief PID 错误检测
 * @note 当前主要用于检测电机堵转等异常状态
 */
void Pid::ErrorHandle() {
  if (fabsf(output_) < max_out_ * 0.001f || fabsf(ref_) < 0.0001f)
    return;
  if (fabsf(ref_ - measure_) / fabsf(ref_) > 0.95f) {
    error_handle.ERRORCount++;
  } else {
    error_handle.ERRORCount = 0;
  }

  if (error_handle.ERRORCount > 500) {
    error_handle.ERRORType = PID_MOTOR_BLOCKED_ERROR;
  }
}
