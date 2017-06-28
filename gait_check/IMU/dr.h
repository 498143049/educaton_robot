#ifndef __DR_H
#define __DR_H

#include "attitude.h"

typedef struct {
    float X; 
    float Y;
    float Z;
} VectorTypedef;


// 根据加速度计信息计算得到的加速度(载体坐标系, 芯片位置)
extern VectorTypedef    DR_AccChip;
// 根据加速度计信息计算得到的加速度(载体坐标系, 轴中点)
extern VectorTypedef    DR_AccCar;
// 根据加速度计信息计算得到的加速度(地理坐标系, 轴中点)
extern VectorTypedef    DR_Acc;
// 估计速度(地理坐标系, 轴中点)
extern VectorTypedef    DR_Speed;
// 估计位置(地理坐标系, 轴中点)
extern VectorTypedef    DR_Position;
// 估计速度(载体坐标系, 轴中点)
extern VectorTypedef    DR_SpeedCar;
// 根据左轮速度与航向角速度计算得到的速度(仅有前进方向上的分量)
extern float            DR_SpeedwL;
// 根据右轮速度与航向角速度计算得到的速度(仅有前进方向上的分量)
extern float            DR_SpeedwR;
// 根据左右轮速度计算得到的速度(仅有前进方向上的分量)
extern float            DR_SpeedLR;
// 根据左右轮速度计算得到的航向角速度(rad/s)
extern float            DR_YawRateLR;
// 根据航向角速度和与估计速度计算的理论向心加速度
extern float            DR_CenterAcc;

// 纯编码器计算的航向角
extern float            DR_EncoderYaw;
// 纯编码器计算的位置(地理坐标系, 轴中点)
extern VectorTypedef    DR_EncoderPosition;
// 偏航角加编码器计算的位置(地理坐标系, 轴中点)
extern VectorTypedef    DR_EncoderYawPosition;

// 运动状态: 静止
extern bool DR_StandStill;
// 运动状态: 倾斜
extern bool DR_Tilt;
// 运动状态: 打滑
extern bool DR_Skid;

void DR_Init(void);

void DR_Update(void);

#endif
