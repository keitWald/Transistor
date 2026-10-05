/**
 * @file leso.h
 * @brief 二阶被控对象的线性扩张状态观测器（LESO，ADRC）
 * @note  高志强带宽法：beta1=3Wc, beta2=3Wc^2, beta3=Wc^3
 */
#ifndef __LESO_H
#define __LESO_H

typedef struct {
    float h;      // 控制周期步长
    float b0;     // 控制增益估计值（力矩常数相关）
    float Wc;     // 观测器带宽

    float beta1;
    float beta2;
    float beta3;

    float z1;     // 估算角度
    float z2;     // 估算角速度
    float z3;     // 估算总扰动（摩擦、负载、模型误差）

    float u_last; // 上一周期实际下发的力矩
} LESO_t;

void  LESO_Init(LESO_t *leso, float h, float b0, float Wc);
void  LESO_Update(LESO_t *leso, float y_actual);
float LESO_Get_Disturbance(LESO_t *leso);

#endif /* __LESO_H */
