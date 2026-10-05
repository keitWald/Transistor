/**
 * @file leso.c
 * @brief 线性扩张状态观测器实现（LESO，ADRC）
 */
#include "leso.h"

/**
 * @brief 初始化 LESO，按带宽 Wc 配置极点
 */
void LESO_Init(LESO_t *leso, float h, float b0, float Wc)
{
    leso->h = h;
    leso->b0 = b0;
    leso->Wc = Wc;

    leso->beta1 = 3.0f * Wc;
    leso->beta2 = 3.0f * Wc * Wc;
    leso->beta3 = Wc * Wc * Wc;

    leso->z1 = 0.0f;
    leso->z2 = 0.0f;
    leso->z3 = 0.0f;
    leso->u_last = 0.0f;
}

/**
 * @brief 核心迭代更新（每个控制周期调用一次）
 * @param y_actual 通过 CAN 读取到的电机当前真实角度
 */
void LESO_Update(LESO_t *leso, float y_actual)
{
    float e  = leso->z1 - y_actual;

    float dz1 = leso->z2 - leso->beta1 * e;
    float dz2 = leso->z3 - leso->beta2 * e + leso->b0 * leso->u_last;
    float dz3 = -leso->beta3 * e;

    leso->z1 += dz1 * leso->h;
    leso->z2 += dz2 * leso->h;
    leso->z3 += dz3 * leso->h;
}

/**
 * @brief 获取扰动补偿量（总扰动估计 z3）
 */
float LESO_Get_Disturbance(LESO_t *leso)
{
    return leso->z3;
}
