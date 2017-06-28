#include "main.h"
#include "Encoder.h"

// 左右轮车速
float Encoder_SpeedLeft;
float Encoder_SpeedRight;
// 左右轮该周期内行驶的距离
float Encoder_DistanceLeft;
float Encoder_DistanceRight;
// 左右轮累计行驶
float Encoder_SumLeft;
float Encoder_SumRight;
// 这次测量的周期
float Encoder_Period;

// 单位转换比例, m/位
#define UNIT_RADIO  ((double)0.0000523)
    
static TIM_HandleTypeDef *Left_HTIM;
static TIM_HandleTypeDef *Right_HTIM;

static uint16_t LastLeft;
static uint16_t LastRight;
static double LastTime;

void Encoder_Init(TIM_HandleTypeDef *left_htim, TIM_HandleTypeDef *right_htim) {
    Left_HTIM = left_htim;
    Right_HTIM = right_htim;   
    // 变量初始化
    Encoder_SpeedLeft = 0;
    Encoder_SpeedRight = 0;
    Encoder_DistanceLeft = 0;
    Encoder_DistanceRight = 0;
    Encoder_SumLeft = 0;
    Encoder_SumRight = 0;
    Encoder_Period = 0.001;
    LastLeft = 0;
    LastRight = 0;
    // 启动编码器计数
    LastTime = MY_GetClock();
    HAL_TIM_Encoder_Start(Left_HTIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(Right_HTIM, TIM_CHANNEL_ALL);
}

void Encoder_Measure(void) {
    float t = MY_GetClock();
    uint16_t left = __HAL_TIM_GET_COUNTER(Left_HTIM);
    uint16_t right = __HAL_TIM_GET_COUNTER(Right_HTIM);
    
    int16_t delta_left, delta_right;
    Encoder_Period = t - LastTime;
    delta_left = left - LastLeft;
    delta_right = right - LastRight;
    
    Encoder_DistanceLeft = delta_left * UNIT_RADIO;
    Encoder_DistanceRight = delta_right * UNIT_RADIO*0.9;
    Encoder_SpeedLeft = Encoder_DistanceLeft / Encoder_Period;
    Encoder_SpeedRight = Encoder_DistanceRight / Encoder_Period;
    Encoder_SumLeft += Encoder_DistanceLeft;
    Encoder_SumRight += Encoder_DistanceRight;
    
    LastTime = t;
    LastLeft = left;
    LastRight = right;
}


