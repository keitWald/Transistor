#ifndef XROBOT_IMU_H
#define XROBOT_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stddef.h>
#include "bsp_uart.h"

#define IMU_PACKET_PREFIX 0xA5

// 关键！确保结构体成员紧凑排列，无填充
typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} Vector3;

typedef struct __attribute__((packed)) {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;  //四元数

typedef struct __attribute__((packed)) {
    float rol;
    float pit;
    float yaw;
} EulerAngles;  //欧拉角，单位：rad

// IMU数据结构定义
typedef struct __attribute__((packed)) {
    uint8_t prefix;
    uint64_t time : 40;
    uint64_t sync : 40;  //时间戳，单位：us
    Quaternion quat_;
    Vector3 gyro_;   //角速度，单位：rad/s
    Vector3 accl_;   //加速度，单位：g
    EulerAngles eulr_;
    uint8_t crc8;
} ImuData_t;

#define IMU_DATA_LENGTH sizeof(ImuData_t)
extern uint8_t imu_dma_rx_buffer[IMU_DATA_LENGTH];  //接收缓冲区

// --- 外部可访问的全局变量 ---
extern ImuData_t g_imu_data;
extern volatile bool g_imu_data_ready;

// --- 外部可调用的接口函数 ---

/**
 * @brief 初始化IMU驱动并启动UART DMA接收
 * @param 
 */
void XRobot_Init();
/**
 * @brief 处理接收到的原始IMU数据流 (由bsp_uart调用)
 * @param rx_buffer 包含原始数据的缓冲区
 * @param size 数据的长度
 */
void IMU_Process_Data(uint8_t *rx_buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* IMU_DRIVER_H */