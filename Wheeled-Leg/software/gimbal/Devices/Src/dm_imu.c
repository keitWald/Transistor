#include "dm_imu.h"
#include <string.h>


dm_imu_t dm_imu;
uint8_t dm_data_rx[dm_imu_rx_len];



void dm_imu_data_unpack(uint8_t* pData)
{
	normal_packet_t normal_packet;
	normal_ext_packet_t ext_packet;
	
	memcpy(&normal_packet,pData+0,19);
	
	
	if(normal_packet.header!=0x55||normal_packet.tail!=0x0A)
		return;
	
	if(normal_packet.reg==0x01)
	{
		dm_imu.accel[0]=normal_packet.data[0];
		dm_imu.accel[1]=normal_packet.data[1];
		dm_imu.accel[2]=normal_packet.data[2];
	}
	
	memcpy(&normal_packet,pData+19,19);
	if(normal_packet.header!=0x55||normal_packet.tail!=0x0A)
		return;
	
	if(normal_packet.reg==0x02)
	{
		dm_imu.gyro[0]=normal_packet.data[0];
		dm_imu.gyro[1]=normal_packet.data[1];
		dm_imu.gyro[2]=normal_packet.data[2];
	}
	
	memcpy(&normal_packet,pData+38,19);
	if(normal_packet.header!=0x55||normal_packet.tail!=0x0A)
		return;
	
	if(normal_packet.reg==0x03)
	{
		dm_imu.roll=normal_packet.data[0];
		dm_imu.pitch=normal_packet.data[1];
		dm_imu.yaw=normal_packet.data[2];
	}
	
	memcpy(&ext_packet,pData+57,23);
	if(ext_packet.header!=0x55||ext_packet.tail!=0x0A)
		return;
	
	if(ext_packet.reg==0x04)
	{
		dm_imu.quaternion[0]=ext_packet.data[0];
		dm_imu.quaternion[1]=ext_packet.data[1];
		dm_imu.quaternion[2]=ext_packet.data[2];
		dm_imu.quaternion[3]=ext_packet.data[3];
	}
	
	
}
void dm_imu_init()
{
	memset(dm_data_rx, 0, sizeof(dm_data_rx));
	if (HAL_UART_Receive_DMA(&huart2, dm_data_rx, dm_imu_rx_len) != HAL_OK)
    {
        Error_Handler();
    }
}
