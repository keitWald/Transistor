/**
 * @file linear_td.c
 * @brief 跟踪微分器实现（ADRC 离散最简形式）
 */
#include "linear_td.h"

/**
 * @brief 初始化跟踪微分器
 * @param td 跟踪微分器结构体
 * @param h  离散步长 (1/频率)
 * @param r  跟踪系数（需满足 r*h <= 0.5）
 */
void Linear_TD_Init(Linear_TD_t *td, float h, float r)
{
    td->x1_pos = 0.0f;
    td->x2_vel = 0.0f;
    td->h = h;
    td->r = r;
}

/**
 * @brief 跟踪微分器更新（临界阻尼二阶差分方程 + 欧拉离散积分）
 * @param td         跟踪微分器结构体
 * @param target_pos 目标位置
 */
void Linear_TD_Update(Linear_TD_t *td, float target_pos)
{
    float dx1 = td->x2_vel;
    float dx2 = -td->r * td->r * (td->x1_pos - target_pos) - 2.0f * td->r * td->x2_vel;

    td->x1_pos += dx1 * td->h;
    td->x2_vel += dx2 * td->h;
}
