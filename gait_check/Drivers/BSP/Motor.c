#include "Motor.h"

#define MAX_COUNT   7200

static TIM_HandleTypeDef *Left_HTIM;
static TIM_HandleTypeDef *Right_HTIM;
static uint32_t LeftForward_Channel;
static uint32_t LeftReverse_Channel;
static uint32_t RightForward_Channel;
static uint32_t RightReverse_Channel;
static float Speed;
static float SpeedDiff;

void Motor_Init(TIM_HandleTypeDef *left, TIM_HandleTypeDef *right) {
    Speed = 0;
    SpeedDiff = 0;
    Left_HTIM = left;
    Right_HTIM = right;
    LeftForward_Channel = TIM_CHANNEL_1;
    LeftReverse_Channel = TIM_CHANNEL_2;
    RightForward_Channel = TIM_CHANNEL_3;
    RightReverse_Channel = TIM_CHANNEL_4;
    Motor_SetDutyLeft(0);
    Motor_SetDutyRight(0);
    HAL_TIM_PWM_Start(Left_HTIM, LeftForward_Channel);
    HAL_TIM_PWM_Start(Left_HTIM, LeftReverse_Channel);
    HAL_TIM_PWM_Start(Right_HTIM, RightForward_Channel);
    HAL_TIM_PWM_Start(Right_HTIM, RightReverse_Channel);
}

void Motor_SetDutyLeft(float duty) {
    if (duty >= 1) {
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftForward_Channel, 7200);
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftReverse_Channel, 0);
    } else if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftForward_Channel, 7200 * duty);
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftReverse_Channel, 0);
    } else if (duty > -1) {
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftReverse_Channel, 7200 * -duty);
    } else if (duty <= -1) {
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftReverse_Channel, 7200);
    } else {
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Left_HTIM, LeftReverse_Channel, 0);
    }
}

void Motor_SetDutyRight(float duty) {
    if (duty >= 1) {
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightForward_Channel, 7200);
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightReverse_Channel, 0);
    } else if (duty >= 0) {
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightForward_Channel, 7200 * duty);
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightReverse_Channel, 0);
    } else if (duty > -1) {
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightReverse_Channel, 7200 * -duty);
    } else if (duty <= -1) {
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightReverse_Channel, 7200);
    } else {
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightForward_Channel, 0);
        __HAL_TIM_SET_COMPARE(Right_HTIM, RightReverse_Channel, 0);
    }
}

float Motor_GetDutyLeft(void) {
    return ((uint16_t)__HAL_TIM_GET_COMPARE(Left_HTIM, LeftForward_Channel) - (uint16_t)__HAL_TIM_GET_COMPARE(Left_HTIM, LeftReverse_Channel)) / 7200.0f;
}

float Motor_GetDutyRight(void) {
    return ((uint16_t)__HAL_TIM_GET_COMPARE(Right_HTIM, RightForward_Channel) - (uint16_t)__HAL_TIM_GET_COMPARE(Right_HTIM, RightReverse_Channel)) / 7200.0f;
}
