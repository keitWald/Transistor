// LqrGains.h
#ifndef LQR_GAINS_H
#define LQR_GAINS_H

#include "lqr_coeffs.h" // 包含由MATLAB生成的系数数据

// 只有在 C++ 编译器下才编译以下内容
#ifdef __cplusplus

class LqrGains {
public:
    /**
     * @brief 构造函数
     */
    LqrGains();

    /**
     * @brief 根据当前的腿长和机器人状态，计算实时 LQR 增益矩阵 K
     * @param leg_len_l 左腿长度 (m)
     * @param leg_len_r 右腿长度 (m)
     *
     * @param off_ground 是否判断为离地
     *
     * @param K_out 计算结果，一个 4x10 的 LQR 增益矩阵
     */
    void calculate(float leg_len_l, float leg_len_r, 
                   bool off_ground, 
                   float K_out[4][10]);

private:
    /**
     * @brief 内部辅助函数，用于计算K矩阵的一行
     * @param gain_type 选择使用哪一套拟合系数 (正常或离地)
     * @param row_index 要计算的是K矩阵的第几行 (0-3)
     * @param leg_len_l 左腿长
     * @param leg_len_r 右腿长
     * @param K_row_out 计算结果，一个包含10个元素的一维数组
     */
    void calculate_K_row(GainType_e gain_type, uint8_t row_index, 
                         float leg_len_l, float leg_len_r, 
                         float K_row_out[10]);
};

#endif // __cplusplus

#endif // LQR_GAINS_H
