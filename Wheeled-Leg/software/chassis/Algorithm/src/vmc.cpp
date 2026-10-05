/**
 *******************************************************************************
 * @file      : vmc.cpp
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
#include "vmc.h"
#include "arm_math.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const float k_m_wheel = 0.612f;//轮的质量
const float k_m_leg=0.788f;
const float length1 = 0.1134f;
const float length2 = 0.135f;
const float length3 = 0.135f;
const float length4 = 0.1134f;
const float length5 = 0.210f;
static const float kLength2Sq = length2 * length2;
static const float kLength3Sq = length3 * length3;
static const float kLength5Over4 = length5 / length4;
static const float kLength4Over5 = length4 / length5;


// 将角度限制在0到2π之间
static inline float WrapTo2Pi(float angle_rad) {
  const float two_pi = 2.0f * PI;
  float wrapped = fmodf(angle_rad, two_pi);
  if (wrapped < 0.0f) {
    wrapped += two_pi;
  }
  return wrapped;
}


// 将角度限制在-π到π之间
static inline float WrapToPi(float angle_rad) {
  const float two_pi = 2.0f * PI;
  float wrapped = fmodf(angle_rad + PI, two_pi);
  if (wrapped < 0.0f) {
    wrapped += two_pi;
  }
  return wrapped - PI;
}
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 根据关节角度,进行腿部运动学正解
 *
 * @note 右侧视图
 *  ___x
 * |   A1  _____ E4
 * |y     /     \
 *     B2 \     /D3
 *         \   /
 *          \./
 *          C5
 * @param p 5连杆和腿的参数
 */
void Vmc::LegCalc() {
	//确定BD两点的横纵坐标
  phi1_ = WrapToPi(phi1_);
  phi4_ = WrapTo2Pi(phi4_);
  float sin_phi1, cos_phi1;
  float sin_phi4, cos_phi4;
  sin_phi1 = arm_sin_f32(phi1_);
  cos_phi1 = arm_cos_f32(phi1_);
  sin_phi4 = arm_sin_f32(phi4_);
  cos_phi4 = arm_cos_f32(phi4_);
  coord_[0] = x_b_ = length1 * cos_phi1;
  coord_[1] = y_b_ = length1 * sin_phi1;
  coord_[4] = x_d_ = length4 * cos_phi4;
  coord_[5] = y_d_ = length4 * sin_phi4;
//计算phi2
  const float dx = x_d_ - x_b_;
  const float dy = y_d_ - y_b_;
  const float bd_sq = dx * dx + dy * dy;
  bd_ = sqrtf(bd_sq);
  a0_ = 2.0f * length2 * dx;
  b0_ = 2.0f * length2 * dy;
  c0_ = kLength2Sq + bd_sq - kLength3Sq;
  const float disc = (a0_ * a0_) + (b0_ * b0_) - (c0_ * c0_);
  const float sqrt_disc = (disc > 0.0f) ? sqrtf(disc) : 0.0f;
  phi2_ = 2.0f * atan2f(b0_ - sqrt_disc, a0_ + c0_) + 2.0f * PI;
	//确定C点的横纵坐标
  float sin_phi2, cos_phi2;
  sin_phi2 = arm_sin_f32(phi2_);
  cos_phi2 = arm_cos_f32(phi2_);
  coord_[2] = x_c_ = kLength5Over4 * (x_b_ + length2 * cos_phi2);
  coord_[3] = y_c_ = kLength5Over4 * (y_b_ + length2 * sin_phi2);

  phi0_ = atan2f(y_c_, x_c_);
  //将phi_0映射到0~2PI之间
  phi0_ = WrapTo2Pi(phi0_);
  l0_ = sqrtf((x_c_ * x_c_) + (y_c_ * y_c_));
  phi3_ = atan2f(y_c_ - y_d_, x_c_ - x_d_);
	//机体倾角得到腿部倾角（与竖直方向夹角）
  theta_ = phi0_ - 0.5f * PI + phi_;
  float sin_theta, cos_theta;
  sin_theta = arm_sin_f32(theta_);
  cos_theta = arm_cos_f32(theta_);
  height_ = l0_ * cos_theta;
 }

/**
 * @brief Inverse kinematics: compute phi1_/phi4_ from target phi0/l0.
 *
 * This uses phi0_cmd_ and l0_cmd_ and writes results into phi1_cmd_/phi4_cmd_.
 * Formula is left as TODO and should be filled in later.
 */
void Vmc::LegInverseCalc() 
{
  const float kEps = 1e-6f;
  float sin_phi0, cos_phi0;
  const float phi0 = WrapTo2Pi(phi0_cmd_);
  sin_phi0 = arm_sin_f32(phi0);
  cos_phi0 = arm_cos_f32(phi0);
  const float kR = kLength4Over5 * l0_cmd_;
  x_c_cmd_ = kR * cos_phi0;
  y_c_cmd_ = kR * sin_phi0;

  const float y_sq = y_c_cmd_ * y_c_cmd_;

  const float x_l1 = x_c_cmd_ + length1;
  const float x_r1 = x_c_cmd_ - length1;
  A_IK_ = x_l1 * x_l1 + y_sq - kLength2Sq;
  B_IK_ = -4.0f * length1 * y_c_cmd_;
  C_IK_ = x_r1 * x_r1 + y_sq - kLength2Sq;

  const float disc1 = (B_IK_ * B_IK_) - (4.0f * A_IK_ * C_IK_);
  if (fabsf(A_IK_) < kEps || disc1 < 0.0f) {
    ik_valid_ = false;
    phi1_cmd_ = 0.0f;
    phi4_cmd_ = 0.0f;
    return;
  }
  const float inv_2a = 1.0f / (2.0f * A_IK_);
  const float u = (-B_IK_ - sqrtf(disc1)) * inv_2a;
  phi1_cmd_ = WrapToPi(2.0f * atan2f(u, 1.0f));

  const float x_l4 = x_c_cmd_ + length4;
  const float x_r4 = x_c_cmd_ - length4;
  D_IK_ = x_l4 * x_l4 + y_sq - kLength3Sq;
  E_IK_ = -4.0f * length4 * y_c_cmd_;
  F_IK_ = x_r4 * x_r4 + y_sq - kLength3Sq;

  const float disc2 = (E_IK_ * E_IK_) - (4.0f * D_IK_ * F_IK_);
  if (fabsf(D_IK_) < kEps || disc2 < 0.0f) {
    ik_valid_ = false;
    phi1_cmd_ = 0.0f;
    phi4_cmd_ = 0.0f;
    return;
  }
  const float inv_2d = 1.0f / (2.0f * D_IK_);
  const float v = (-E_IK_ + sqrtf(disc2)) * inv_2d;
  phi4_cmd_ = WrapTo2Pi(2.0f * atan2f(v, 1.0f));
  ik_valid_ = true;
}

/**
 * @brief Calculate the torque for Vmc.
 */
void Vmc::TorCalc() {
  T1_ = j_[0] * F_ + j_[2] * Tp_;
  T2_ = j_[1] * F_ + j_[3] * Tp_;
}

void Vmc::Jacobian()
{
      // 防止奇异姿态导致除以零
  float s23 = arm_sin_f32(phi2_ - phi3_);
  if (fabsf(s23) < 1e-6f) s23 = (s23 >= 0 ? 1e-6f : -1e-6f);

  const float sin12 = arm_sin_f32(phi1_ - phi2_);
  float s12 = sin12;
  if (fabsf(s12) < 1e-6f) s12 = (s12 >= 0 ? 1e-6f : -1e-6f);

  const float sin34 = arm_sin_f32(phi3_ - phi4_);
  float s34 = sin34;
  if (fabsf(s34) < 1e-6f) s34 = (s34 >= 0 ? 1e-6f : -1e-6f);

  float sin03, cos03;
  float sin02, cos02;
  sin03 = arm_sin_f32(phi0_ - phi3_);
  cos03 = arm_cos_f32(phi0_ - phi3_);
  sin02 = arm_sin_f32(phi0_ - phi2_);
  cos02 = arm_cos_f32(phi0_ - phi2_);

  const float inv_s23 = 1.0f / s23;
  const float inv_s12 = 1.0f / s12;
  const float inv_s34 = 1.0f / s34;
  const float inv_l0 = 1.0f / l0_;

  const float kL15Over4 = (length1 * length5) / length4;
  const float kL4Over15 = length4 / (length1 * length5);
  const float inv_l5 = 1.0f / length5;

  j_[0] = -kL15Over4 * sin03 * sin12 * inv_s23;
  j_[1] = -length5 * sin02 * sin34 * inv_s23;
  j_[2] = -kL15Over4 * cos03 * sin12 * inv_l0 * inv_s23;
  j_[3] = -length5 * cos02 * sin34 * inv_l0 * inv_s23;

  inv_j_[0] = -kL4Over15 * cos02 * inv_s12;
  inv_j_[1] = kL4Over15 * l0_ * sin02 * inv_s12;
  inv_j_[2] = cos03 * inv_l5 * inv_s34;
  inv_j_[3] = -l0_ * sin03 * inv_l5 * inv_s34;

}
void Vmc::LegForceCalc() {
  mea_F_ = inv_j_[0] * mea_t1_ + inv_j_[2] * mea_t2_;
  mea_Tp_ = inv_j_[1] * mea_t1_ + inv_j_[3] * mea_t2_;
  float sin_theta, cos_theta;
  sin_theta = arm_sin_f32(theta_);
  cos_theta = arm_cos_f32(theta_);
  const float inv_l0 = 1.0f / l0_;
  p_ = mea_F_ * cos_theta - (mea_Tp_ * sin_theta) * inv_l0;
  F_N_ = p_ + k_m_wheel * (9.8f +ddot_z_w_); //用于离地检测
}
//计算腿部相关角度的变化率
void Vmc::DerivateCalc()
{

    observer_dt_ = DWT_GetDeltaT(&dwt_cnt_observer);
    const float kEps   = 1e-6f;

    // --- 1. 运动学解算 (保持不变) ---
    float s23 = arm_sin_f32(phi3_-phi2_);
    if (fabsf(s23) < kEps) s23 = (s23 >= 0.0f ? kEps : -kEps);   // 奇异保护
    
    w_phi2_ = (length1*w_phi1_*arm_sin_f32(phi1_-phi3_) + length4*w_phi4_*arm_sin_f32(phi3_-phi4_))/
          s23/length2; 
    
    float sin_phi1, cos_phi1;
    float sin_phi2, cos_phi2;
    sin_phi1 = arm_sin_f32(phi1_);
    cos_phi1 = arm_cos_f32(phi1_);
    sin_phi2 = arm_sin_f32(phi2_);
    cos_phi2 = arm_cos_f32(phi2_);
    dot_xb_ = -length1*w_phi1_*sin_phi1;
    dot_yb_ = length1*w_phi1_*cos_phi1;
    dot_xc_ = kLength5Over4*(dot_xb_ - length2 * w_phi2_ * sin_phi2);
    dot_yc_ = kLength5Over4*(dot_yb_ + length2 * w_phi2_ * cos_phi2);
    
    const float inv_l0 = 1.0f / l0_;
    const float inv_l0_sq = inv_l0 * inv_l0;
    w_phi0_ = (x_c_ * dot_yc_ - y_c_ * dot_xc_) * inv_l0_sq;
    v_l0_   = (x_c_ * dot_xc_ + y_c_ * dot_yc_) * inv_l0;

    // --- 2. 计算 w_theta 原始值 ---
    // w_theta_ = w_phi0_ + w_phi_; 
    float w_theta_raw = w_phi0_ + w_phi_; 

    // --- 3. [新增] w_theta 的低通滤波 ---
    // 系数建议：0.0 不滤波，1.0 完全不动。
    // 速度项建议 0.4 - 0.7 之间，既要平滑又要保证延迟不过大
    // The later 0.6 filter added phase lag inside the fitted LQR loop.
    static const float w_filter_alpha = 0.0f;
    
    // 一阶低通滤波公式: Output = alpha * Last_Output + (1 - alpha) * Input
    // 注意：利用 last_w_theta_ (上一时刻的值) 来进行滤波
    w_theta_ = w_filter_alpha * last_w_theta_ + (1.0f - w_filter_alpha) * w_theta_raw;

    // --- 4. 使用滤波后的 w_theta 计算后续物理量 ---
    // 注意：v_height_ 依赖 w_theta_，所以必须在滤波后计算
    float sin_theta, cos_theta;
    sin_theta = arm_sin_f32(theta_);
    cos_theta = arm_cos_f32(theta_);
    v_height_ = v_l0_ * cos_theta - l0_ * sin_theta * w_theta_;

    // --- 5. 差分计算加速度 (保持原逻辑，但现在输入更平滑了) ---
    static float alpha = 0.81f;  // 加速度的滤波系数通常要大一点(0.8-0.9)，因为二阶导噪声极大
    
    dot_v_l0_ = alpha * dot_v_l0_ + (1.0f - alpha) * (v_l0_ - last_v_l0_) / observer_dt_;
    
    // 注意：这里计算 dotw_theta_ 时，使用的是滤波后的 w_theta_ 和上一时刻的 last_w_theta_
    // 这样算出来的角加速度也会自然变得更平滑
    dotw_theta_ = alpha * dotw_theta_ + (1.0f - alpha) * (w_theta_ - last_w_theta_) / observer_dt_;

    const float w_theta_sq = w_theta_ * w_theta_;
    ddot_z_w_ = ddot_z_M_ - dot_v_l0_ * cos_theta +
              2.0f * v_l0_ * w_theta_ * sin_theta +
              l0_ * dotw_theta_ * sin_theta +
              l0_ * w_theta_sq * cos_theta;

    // --- 6. 更新历史值 ---
    last_w_theta_ = w_theta_; // 将滤波后的值存入历史，供下一帧使用
    last_v_l0_    = v_l0_;
}



