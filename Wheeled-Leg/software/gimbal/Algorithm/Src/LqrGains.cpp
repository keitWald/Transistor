// LqrGains.cpp
#include "LqrGains.h"


LqrGains::LqrGains()
{
    // 构造函数可以为空，因为数据都是编译时确定的常量
}

void LqrGains::calculate_K_row(GainType_e gain_type, uint8_t row_index, 
                               float leg_len_l, float leg_len_r, 
                               float K_row_out[10])
{
    // 根据选择的工况，决定使用哪个系数矩阵
    const float (*coeffs_ptr)[K_COEFFS] = (gain_type == GAINS_NORMAL) ? K_coeffs_normal : K_coeffs_off_ground;

    // 预计算腿长的多项式项
    const float l_sq_l = leg_len_l * leg_len_l;
    const float l_sq_r = leg_len_r * leg_len_r;
    const float l_lr = leg_len_l * leg_len_r;

    // 计算K矩阵指定行的10个元素
    for (uint8_t j = 0; j < 10; ++j) {
        int coeff_row = row_index * 10 + j;
        const float* c = coeffs_ptr[coeff_row]; // 获取当前K_ij对应的6个拟合系数
        
        // 使用拟合多项式计算 K_ij
        K_row_out[j] = c[0] + c[1] * leg_len_l + c[2] * leg_len_r + c[3] * l_sq_l + c[4] * l_sq_r + c[5] * l_lr;
    }
}

void LqrGains::calculate(float leg_len_l, float leg_len_r,
                       bool off_ground,
                       float K_out[4][10])
{
    if (!off_ground) {
        // Normal: compute full K matrix
        for (uint8_t i = 0; i < 4; ++i) {
            calculate_K_row(GAINS_NORMAL, i, leg_len_l, leg_len_r, K_out[i]);
        }
        return;
    }

    // Off-ground: keep only K35~K38 and K45~K48 (1-based)
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 10; ++j) {
            K_out[i][j] = 0.0f;
        }
    }

    float temp_row[10];
    // K35~K38 -> row index 2, col index 4..7
    calculate_K_row(GAINS_OFF_GROUND, 2, leg_len_l, leg_len_r, temp_row);
    for (uint8_t k_idx = 4; k_idx <= 7; ++k_idx) {
        K_out[2][k_idx] = temp_row[k_idx];
    }
    // K45~K48 -> row index 3, col index 4..7
    calculate_K_row(GAINS_OFF_GROUND, 3, leg_len_l, leg_len_r, temp_row);
    for (uint8_t k_idx = 4; k_idx <= 7; ++k_idx) {
        K_out[3][k_idx] = temp_row[k_idx];
    }
}