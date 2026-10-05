/**
 * @file linear_td.h
 * @brief 跟踪微分器（ADRC 离散最简形式）
 * @note  平滑目标轨迹，输出位置 P_des 与速度 V_des；需满足 r*h <= 0.5
 */
#ifndef __LINEAR_TD_H
#define __LINEAR_TD_H

typedef struct
{
    float x1_pos;   // 平滑位置 P_des
    float x2_vel;   // 平滑速度 V_des
    float h;        // 离散步长 (1/频率)
    float r;        // 跟踪系数（需满足 r*h <= 0.5）
} Linear_TD_t;

void Linear_TD_Init(Linear_TD_t *td, float h, float r);
void Linear_TD_Update(Linear_TD_t *td, float target_pos);

#endif /* __LINEAR_TD_H */
