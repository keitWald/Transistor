/**
 *******************************************************************************
 * @file      : pid.h
 * @brief     : PID 控制器头文件
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
/* 防止重复包含 --------------------------------------------------------------*/
#ifndef __PID_H_
#define __PID_H_

/* 头文件包含 ----------------------------------------------------------------*/
#include "arm_math.h"
#include "stdint.h"
/* 宏定义 --------------------------------------------------------------------*/
/* 常量定义 ------------------------------------------------------------------*/
#define yaw_O_p 0.083f
#define yaw_O_i 0.00001f
#define yaw_O_d 0.0f
#define yaw_I_p 1050.0f
#define yaw_I_i 0.12f
#define yaw_I_d 1000.0f
#define yaw_O_Aim_p 2.082f
#define yaw_O_Aim_i 0.00001f
#define yaw_O_Aim_d 1.0f
#define yaw_O_Aim_f 0.0f
#define yaw_I_Aim_p 1100.0f
#define yaw_I_Aim_i 60.0f
#define yaw_I_Aim_d 950.0f
#define yaw_I_Aim_f 100.0f

typedef enum {
  PID_IMPROVE_NONE = 0b00000000,                 // 0000 0000
  PID_INTEGRAL_LIMIT = 0b00000001,               // 0000 0001
  PID_DERIVATIVE_ON_MEASUREMENT = 0b00000010,    // 0000 0010
  PID_TRAPEZOID_INTEGRAL = 0b00000100,           // 0000 0100
  PID_PROPORTIONAL_ON_MEASUREMENT = 0b00001000,  // 0000 1000
  PID_OUTPUT_FILTER = 0b00010000,                // 0001 0000
  PID_CHANGING_INTEGRATION_RATE = 0b00100000,    // 0010 0000
  PID_DERIVATIVE_FILTER = 0b01000000,            // 0100 0000
  PID_ERROR_HANDLE = 0b10000000,                 // 1000 0000
} PidImprovement;

typedef enum errorType_e {
  PID_ERROR_NONE = 0x00U,
  PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType;

typedef struct {
  uint64_t ERRORCount;
  ErrorType ERRORType;
} PidErrorHandler;
#ifdef __cplusplus
/* 类型定义 ------------------------------------------------------------------*/
class Pid {
 public:
  // PID 基本参数初始化
  void Init(float _kp, float _ki, float _kd,float _max_out, float _dead_band) {
    kp_ = _kp;
    ki_ = _ki;
    kd_ = _kd;
    kf_ = 0.0f;
    max_out_ = _max_out;
    dead_band_ = _dead_band;
    improve_ = PID_IMPROVE_NONE;
    integral_limit_ = 0.0f;
    coef_a_ = 0.0f;
    coef_b_ = 0.0f;
    output_lpf_rc_ = 0.0f;
    derivative_lpf_rc_ = 0.0f;
    error_handle.ERRORCount = 0;
    error_handle.ERRORType = PID_ERROR_NONE;
    Clear();
  };

  // PID 改进项参数配置
  void Inprovement(uint8_t _improve, float _integral_limit, float _coef_a,
                   float _coef_b, float _output_lpf_rc,
                   float _derivative_lpf_rc) {
    improve_ = _improve;
    integral_limit_ = _integral_limit;
    coef_a_ = _coef_a;
    coef_b_ = _coef_b;
    output_lpf_rc_ = _output_lpf_rc;
    derivative_lpf_rc_ = _derivative_lpf_rc;
  };

  void SetRef(float _ref) { ref_ = _ref; };

  void SetMeasure(float _measure) { measure_ = _measure; };

  float GetOutput() { return output_; }
  // 清除运行时状态，保留 PID 参数配置
  void Clear();
  // 计算当前 PID 输出
  float Calculate();
  // 梯形积分
  void TrapezoidIntergral();
  // 变速率积分
  void ChangingIntegratioRate();
  // 积分限幅
  void IntegralLimit();
  // 测量微分
  void DerivativeOnMeasurement();
  // 微分项低通滤波
  void DerivativeFilter();
  // 输出低通滤波
  void OutputFilter();
  // 输出限幅
  void OutputLimit();
  // 错误检测
  void ErrorHandle();

 private:
  // PID 基本参数
  float kp_;
  float ki_;
  float kd_;
  float kf_;
  float max_out_;
  float dead_band_;

  // PID 改进选项及错误处理
  uint8_t improve_;
  PidErrorHandler error_handle;

  // 改进项相关参数
  float integral_limit_;
  float coef_a_;
  float coef_b_;
  float output_lpf_rc_;
  float derivative_lpf_rc_;

  // 状态量
  float ref_, measure_, last_measure_;
  float err_, last_err_;
  float p_out_, i_out_, d_out_, last_d_out_, i_term_, last_i_term_;
  float output_, last_output_;

  // 运行时参数
  uint32_t DWT_CNT;
  float dt;
};
/* 变量声明 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
#endif
#endif /* __PID_H_ */
