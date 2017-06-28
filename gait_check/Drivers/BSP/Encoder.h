#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f1xx_hal.h"

// 左右轮车速
extern float Encoder_SpeedLeft;
extern float Encoder_SpeedRight;
// 左右轮该周期内行驶的距离
extern float Encoder_DistanceLeft;
extern float Encoder_DistanceRight;
// 左右轮累计行驶
extern float Encoder_SumLeft;
extern float Encoder_SumRight;
// 这次测量的周期
extern float Encoder_Period;

void Encoder_Init(TIM_HandleTypeDef *left_htim, TIM_HandleTypeDef *right_htim);

void Encoder_Measure(void);

#endif
