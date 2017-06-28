#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f1xx_hal.h"

// �����ֳ���
extern float Encoder_SpeedLeft;
extern float Encoder_SpeedRight;
// �����ָ���������ʻ�ľ���
extern float Encoder_DistanceLeft;
extern float Encoder_DistanceRight;
// �������ۼ���ʻ
extern float Encoder_SumLeft;
extern float Encoder_SumRight;
// ��β���������
extern float Encoder_Period;

void Encoder_Init(TIM_HandleTypeDef *left_htim, TIM_HandleTypeDef *right_htim);

void Encoder_Measure(void);

#endif
