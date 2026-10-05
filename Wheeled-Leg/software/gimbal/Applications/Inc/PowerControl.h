/**
 * @file PowerControl.c
 * @author Why
 * @brief ����������������ƹ���֮��
 * @version 0.1
 * @date 2023-08-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __POWERCONTROL_H
#define __POWERCONTROL_H

#include "main.h"

/******�ؼ��趨��pid��3508��������ǰ�ĸ���һһ��Ӧ���ĸ�����
*******����������ط��õ�************************************/

/* ת�ص�����ȡֵ��ʵ��ת��֮���ת����������λN��m/A ����ʽ���� = TC * I��*/
#define Torque_Constant 0.0000190703f
#define It_Constant 52437.56f  
/* ��е���ʳ�������ʽ��Pm = ��*w / 9.55 = ��*w*MPC */
#define MachinePower_Cnstant 0.104712042 
/* ʵ�ʹ��ʾ��鹫ʽ��������Ҫʵ�ʲ�����ϣ���ʽ��Pin = Pm + Kw * w^2 +K�� * ��^2 + A */
#define Kw_d 0
#define Kt_d 0
#define A_d 0

#define Div_2Kt_d 0

#define PowerControl_FunGroundInit           \
    {                              		\
		&PowerControl_Handle,           \
    }

#define PowerControl_DataGroundInit \
    {                      \
		0,                 \
		0,                 \
		0,                 \
    }

typedef struct
{
	/* �������ֵ */
	float OverLimit_Ratio;         // �������ʵı���
	
	/* ����ϵͳ��ֵ */
	uint16_t Power_Limit;          // ���ƹ���
	float Power_Observe_ByJudge;   //
	float Cap_U;                   //����ϵͳ�������Ĺ���ֵ
} PowerControl_Data_t;

typedef struct
{
    void (*PowerControl_Handle)(void);
} PowerControl_Fun_t;

extern PowerControl_Fun_t PowerControl_Fun;
extern PowerControl_Data_t PowerControl_Data;

#endif /*__POWERCONTROL_H*/
