/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @frame 帧头 0x42 0x52  —— ASCII是BR
 *  	  对于弹道数据，第三个字节是命令码,0xCD；
 *  	  第四个字节是数据帧的长度；
 *  	  之后是数据帧，弹道有关的有一个字节
 *		  下位机发给上位机是射速，上位机发给下位机是云台100倍的俯仰*弧度*；
 *	  	  帧尾 一字节的CRC8
 * @version 0.2
 * @date 2024-1-28
 *
 */
#include "Protocol_UpperComputer.h"
#include "BSP_BoardCommunication.h"
#include "J4310_Motor.h"
#include <stdint.h>

float Auto_Aim_Yaw;
float Auto_Aim_Pitch;
bool Fire_Flag = 0;
positionpid_t Auto_Aim_PID;

 /**
 * @brief  UpperCom下位机与上位机通信，向上位机发送信息。使用USB
 * @note   对应 NUC 端 ReceivePacket
 * @param  void
 * @retval void
 */
void UpperCom_Send_To_Up(void)
{
	uint8_t UpperCom_Send_Buffer[UpperCom_MAX_BUF];

	/*获取弹速*/
	float bullet_velocity = 27.0f;
	if (ControlMes.Speed_Bullet >= 5)
		bullet_velocity = ControlMes.Speed_Bullet;
	static uint8_t mark = 0;

	/* 先录入帧头 */
	UpperCom_Send_Buffer[0] = 0x42;
	UpperCom_Send_Buffer[1] = 0x52;
	UpperCom_Send_Buffer[2] = PROTO_VER;
	UpperCom_Send_Buffer[3] = PROTO_SEND_DATALEN; // 数据包包含的字节数



	if(mark++ >= 200)
		mark = 0;
	memcpy(&UpperCom_Send_Buffer[4], &bullet_velocity, sizeof(bullet_velocity));
	memcpy(&UpperCom_Send_Buffer[8], &J4310s_Pitch.realAngle, sizeof(J4310s_Pitch.realAngle));
	memcpy(&UpperCom_Send_Buffer[12], &ControlMes.yaw_realAngle_double, sizeof(ControlMes.yaw_realAngle_double));//yaw轴电机编码器的角度
	memcpy(&UpperCom_Send_Buffer[20], &ControlMes.yaw_omega, sizeof(ControlMes.yaw_omega)); //yaw轴电机编码器读到的角速度 rad/s
	memcpy(&UpperCom_Send_Buffer[24], &ControlMes.chassis_imu_yaw, sizeof(ControlMes.chassis_imu_yaw)); // 底盘IMU的yaw轴角度 0-2pai
	memcpy(&UpperCom_Send_Buffer[28], &ControlMes.chassis_imu_omega, sizeof(ControlMes.chassis_imu_omega));
	memcpy(&UpperCom_Send_Buffer[32], &mark, sizeof(mark)); // 记录帧数
	memcpy(&UpperCom_Send_Buffer[33], &ControlMes.tnndcolor, sizeof(ControlMes.tnndcolor)); // 己方颜色，1为红，2为蓝
	memcpy(&UpperCom_Send_Buffer[34], &ControlMes.AutoAimFlag, sizeof(ControlMes.AutoAimFlag)); // 自瞄开关,1为开，0为关
	memcpy(&UpperCom_Send_Buffer[35], &ControlMes.chassis_yaw_temp, sizeof(ControlMes.chassis_yaw_temp)); // 底盘yaw温度
    Append_CRC8_Check_Sum(UpperCom_Send_Buffer, 5 + UpperCom_Send_Buffer[3]); // 5+x，x代表数据包包含的数据字节数。
	CDC_Transmit_HS(UpperCom_Send_Buffer, sizeof(UpperCom_Send_Buffer)); // usb发送
	memset(UpperCom_Send_Buffer, 0, UpperCom_MAX_BUF);//清空发送帧，用于下一次发送
}

/**
 * @brief  UpperCom下位机与上位机通信，接受上位机的信息。使用USB
 * @note   对应 NUC 端 SendPacket
 * @param  *Rec 接收到的一帧数据
 * @retval void
 */
void UpperCom_Receive_From_Up(uint8_t Rec[])
{
	/* 先检验帧头 */
	if (Rec[0] != 0x42 || Rec[1] != 0x52)
		return;
	if (Rec[3] != PROTO_RECV_DATALEN)
		return;
	/* 再根据CRC校验 */
	if(!Verify_CRC8_Check_Sum(Rec, 5 + Rec[3]))
		return;

	switch (Rec[2])
	{
	case 0x02:
		ControlMes.NUC_AutoAimFlag = Rec[4];
		Fire_Flag = Rec[5];
		memcpy(&ControlMes.Auto_Aim_Pitch, &Rec[6],4);
		ControlMes.yaw_torque_only_mode = Rec[10];
		memcpy(&ControlMes.yaw_target_angle, &Rec[11],8);
		memcpy(&ControlMes.yaw_target_velocity, &Rec[19],4);
		memcpy(&ControlMes.yaw_torque, &Rec[23],4);
		// yaw_torque 限幅 ±1
		if (ControlMes.yaw_torque > 1.0f)
			ControlMes.yaw_torque = 1.0f;
		else if (ControlMes.yaw_torque < -1.0f)
			ControlMes.yaw_torque = -1.0f;
		break;
	default:
		return;
	}
}
