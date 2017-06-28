#ifndef __MOTOR_H

#include "stm32f1xx_hal.h"

void Motor_Init(TIM_HandleTypeDef *left, TIM_HandleTypeDef *right);
void Motor_SetDutyLeft(float duty);
void Motor_SetDutyRight(float duty);
float Motor_GetDutyLeft(void);
float Motor_GetDutyRight(void);

#endif
