/**
 * @file FuzzyPID.h
 * @author Why/xyz
 * @brief 
 * @version 0.1
 * @date 2024-10-21
 *
 */
#ifndef __FUZZY_PID_H
#define __FUZZY_PID_H
#include "stm32h7xx_hal.h"

#define FUZZYPID_Pitch_GroupInit \
	{                            \
		0,                       \
		0,                       \
		0,                       \
		1000,                    \
		-1000,                   \
		0.f,                     \
		0.0f,                    \
		0,                       \
	}

typedef struct
{
	float delta_kp; 
	float delta_ki; 
	float delta_kd; 

	float error_maximum; 
	float error_minimum; 

	float qKp; 
	float qKi; 
	float qKd; 

	float error_map[2];

	float error_membership_degree[2];
	int8_t error_membership_index[2]; 

	float d_error_membership_degree[2]; 
	int8_t d_error_membership_index[2];

} FUZZYPID_Data_t;
extern void Linear_Quantization(FUZZYPID_Data_t *PID, float thisError, float lastError, float *qValue);
extern void Membership_Calc(float *ms, float qv, int8_t *index);
extern void FuzzyComputation(FUZZYPID_Data_t *PID, float thisError, float lastError);
extern void fuzzy_init(FUZZYPID_Data_t *PID, float _maximum, float _minimum, float _qkp, float _qki, float _qkd);

extern FUZZYPID_Data_t fuzzy_pid_shoot_l;
extern FUZZYPID_Data_t fuzzy_pid_shoot_r;
extern FUZZYPID_Data_t fuzzy_pid_bullet_v;
extern FUZZYPID_Data_t fuzzy_pid_bullet_l;
extern FUZZYPID_Data_t fuzzy_pid_pitch_in;
extern FUZZYPID_Data_t fuzzy_pid_pitch_out;
#endif
