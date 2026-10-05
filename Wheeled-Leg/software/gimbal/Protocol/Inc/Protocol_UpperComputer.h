/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @version 0.1
 * @date 2023-10-02
 *
 */

#ifndef PROTOCOL_UPPERCOMPUTER_H
#define PROTOCOL_UPPERCOMPUTER_H

#include "Protocol_CRC.h"
#include "usart.h"
#include "PID.h"
#include "BSP_BoardCommunication.h"
#include "FrictionWheel.h"
#include "Cloud_Control.h"
#include "usbd_cdc_if.h"

void UpperCom_Receive_From_Up(uint8_t Rec[]);
void UpperCom_Send_To_Up(void);

// ── 新协议帧格式 ──
// STM32 ← NUC (接收): 帧头 0x42 0x52 + 版本 0x01 + 数据长度 23 + 数据区 + CRC8 = 28字节
// STM32 → NUC (发送): 帧头 0x42 0x52 + 版本 0x01 + 数据长度 32 + 数据区 + CRC8 = 37字节

#define PROTO_VER          0x02
#define PROTO_RECV_SIZE    28    // 接收帧总长 (NUC→STM32)
#define PROTO_SEND_SIZE    37    // 发送帧总长 (STM32→NUC)
#define PROTO_RECV_DATALEN 23
#define PROTO_SEND_DATALEN 32
#define UpperCom_MAX_BUF   37

// #define UpperCom_MAX_BUF 25
// #define Test_Pitch_SEN 22.75556
// #define Test_Yaw_SEN 22.75556 // 8192/360 = 22.75556

extern positionpid_t Auto_Aim_PID;
extern float Auto_Aim_Pitch;
extern bool Fire_Flag;
#endif
