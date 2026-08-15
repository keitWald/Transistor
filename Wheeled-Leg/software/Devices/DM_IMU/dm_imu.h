#ifndef __DM_IMU_H
#define __DM_IMU_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "bsp_uart.h"

#define dm_imu_rx_len 80



typedef struct
{
	float accel[3];
	float gyro[3];
	float roll;
	float pitch;
	float yaw;
	float quaternion[4];

}dm_imu_t;


#pragma pack(1)
typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[3];
	uint16_t crc;
	uint8_t tail;

}normal_packet_t;
#pragma pack()



#pragma pack(1)
typedef struct
{
	uint8_t header;
	uint8_t tag;
	uint8_t slave_id;
	uint8_t reg;
	float data[4];
	uint16_t crc;
	uint8_t tail;

}normal_ext_packet_t;
#pragma pack()


void dm_imu_data_unpack(uint8_t* pData);
void dm_imu_init();
extern uint8_t dm_data_rx[dm_imu_rx_len];
extern dm_imu_t dm_imu;
#ifdef __cplusplus
}
#endif
#endif