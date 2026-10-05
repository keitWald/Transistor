/**
 *******************************************************************************
 * @file      : vmc.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VMC_H_
#define __VMC_H_

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "bsp_dwt.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief The Vmc class represents a virtual machine controller.
 *
 * This class provides methods for calculating leg and torso movements,
 * setting and getting various parameters, and storing internal state variables.
 */
 #ifdef __cplusplus
class Vmc {
 public:
  void LegCalc();
  void LegInverseCalc();
  void Jacobian();
  void TorCalc();
  void LegForceCalc();
  void DerivateCalc();  //计算phi0和l0的变化率
  void SetBodyData(const float _phi, const float _acc_z, const float _w_phi_) {
    phi_ = _phi;
    ddot_z_M_ = _acc_z;
    w_phi_ = _w_phi_;
  }
  void SetLegData(const float _phi1, const float _w_phi1, const float _phi4,
                  const float _w_phi4, const float _t1, const float _t2) {
    phi1_ = _phi1;
    w_phi1_ = _w_phi1;
    phi4_ = _phi4;
    w_phi4_ = _w_phi4;
    mea_t1_ = _t1;
    mea_t2_ = _t2;
  };

  void SetTor(const float _F, const float _Tp) {
    F_ = _F;
    Tp_ = _Tp;
  };
  // 参考山海机甲：直接设置关节力矩(用于倒地自起 LengthLQR 控制)
  void SetDirectJointTor(const float _T1, const float _T2) {
    T1_ = _T1;
    T2_ = _T2;
  }
 // 设置目标位置进行逆运动学解算
  void SetLegIKTarget(const float _phi0, const float _l0) {
    phi0_cmd_ = _phi0;
    l0_cmd_ = _l0;
  }
  float GetT1() { return T1_; };
  float GetT2() { return T2_; };
  float GetTheta() { return theta_; };
  float GetDotTheta() { return w_theta_; };
  float GetLegLen() { return l0_; };
  float GetLegSpeed() { return v_l0_; };
  float GetPhi0() { return phi0_; };
  float GetPhi1() { return phi1_; };
  float GetPhi4() { return phi4_; };
  float GetWPhi1() { return w_phi1_; };
  float GetWPhi4() { return w_phi4_; };
  float GetPhi2Speed() { return w_phi2_; };
  float GetForceNormal() { return F_N_; };
  float GetPhi0Speed() { return w_phi0_; };
  float GetPhi1IK() { return phi1_cmd_; };
  float GetPhi4IK() { return phi4_cmd_; };
  bool GetIKValid() { return ik_valid_; };
  float observer_dt_;
  uint32_t dwt_cnt_observer;
 private:
  float phi_, phi0_, phi1_, phi4_, w_phi1_, w_phi4_, l0_, w_phi_;
  float phi0_cmd_, l0_cmd_, phi1_cmd_, phi4_cmd_,  x_c_cmd_, y_c_cmd_;
  float A_IK_, B_IK_, C_IK_, D_IK_, E_IK_, F_IK_;
  bool ik_valid_;
  float theta_, height_, w_theta_, v_height_, dotw_theta_, dotv_height_,
      w_phi2_, v_l0_, ddot_z_w_, dot_v_l0_, w_phi0_, ddot_z_M_;
  float T1_, T2_, F_, Tp_, mea_t1_, mea_t2_, mea_F_, mea_Tp_, F_N_, P_;
  float x_b_, y_b_, x_c_, y_c_, x_d_, y_d_, p_;
  float coord_[6];  // xb yb xc yc xd yd
  float last_w_theta_, last_v_l0_;
  float phi2_, phi3_;
  float a0_, b0_, bd_, c0_;
  float j_[4], inv_j_[4];
  float dot_xb_, dot_yb_, dot_xc_, dot_yc_;
  float A1_;

};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif
#endif /* __VMC_H_ */
