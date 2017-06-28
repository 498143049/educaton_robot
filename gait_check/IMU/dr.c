#include "dr.h"
#include "SEGGER_RTT.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

// 重力加速度定义
#define GRAVITY     (9.8f)
// 车体结构信息
#define d           (0.29f)     // 两轮间距, 单位: m
#define rmsinm      (0.145f)    // rm*sin(两轮中心指向右轮的射线旋转到两轮中心到IMU的射线, 所需的转角)
#define rmcosm      (0.03f)     // rm*cos(两轮中心指向右轮的射线旋转到两轮中心到IMU的射线, 所需的转角)
#define rm          (0.148f)    // IMU到两轮中心距离, 单位: m
// 传感器数据来源定义
#include "main.h"
#include "Encoder.h"
#define CURRENT_CLOCK       MY_GetClock()       // 高精度(微秒级)时间, 单位: s
#define WHEEL_SPEED_LEFT    Encoder_SpeedLeft   // 左轮速, 单位: m/s
#define WHEEL_SPEED_RIGHT   Encoder_SpeedRight  // 右轮速, 单位: m/s
#define WHEEL_SPEED_PERIOD  Encoder_Period      // 轮速采样周期, 单位: s

// 根据加速度计信息计算得到的加速度(载体坐标系, 芯片位置)
VectorTypedef    DR_AccChip;
// 根据加速度计信息计算得到的加速度(载体坐标系, 轴中点)
VectorTypedef    DR_AccCar;
// 根据加速度计信息计算得到的加速度(地理坐标系, 轴中点)
VectorTypedef    DR_Acc;
// 估计速度(地理坐标系, 轴中点)
VectorTypedef    DR_Speed;
// 估计位置(地理坐标系, 轴中点)
VectorTypedef    DR_Position;
// 估计速度(载体坐标系, 轴中点)
VectorTypedef    DR_SpeedCar;
// 根据左轮速度与航向角速度计算得到的速度(仅有前进方向上的分量)
float            DR_SpeedwL;
// 根据右轮速度与航向角速度计算得到的速度(仅有前进方向上的分量)
float            DR_SpeedwR;
// 根据左右轮速度计算得到的速度(仅有前进方向上的分量)
float            DR_SpeedLR;
// 根据左右轮速度计算得到的航向角速度(rad/s)
float            DR_YawRateLR;
// 根据航向角速度和与估计速度计算的理论向心加速度
float            DR_CenterAcc;

// 纯编码器计算的航向角
float            DR_EncoderYaw;
// 纯编码器计算的位置(地理坐标系, 轴中点)
VectorTypedef    DR_EncoderPosition;
// 偏航角加编码器计算的位置(地理坐标系, 轴中点)
VectorTypedef    DR_EncoderYawPosition;

// 运动状态: 静止
bool DR_StandStill;
// 运动状态: 倾斜
bool DR_Tilt;
// 运动状态: 打滑
bool DR_Skid;

#define Calibration_Limit   (0.3f)

#define CONSTRAIT(x,min,max)    ((x>max)?max:(x<min)?min:x)

void DR_Init(void) {
    DR_Acc.X = 0;
    DR_Acc.Y = 0;
    DR_Acc.Z = 0;
    DR_Speed.X = 0;
    DR_Speed.Y = 0;
    DR_Speed.Z = 0;
    DR_Position.X = 0;
    DR_Position.Y = 0;
    DR_Position.Z = 0;
    DR_EncoderYaw = 0;
    DR_EncoderPosition.X = 0;
    DR_EncoderPosition.Y = 0;
    DR_EncoderPosition.Z = 0;
    DR_EncoderYawPosition.X = 0;
    DR_EncoderYawPosition.Y = 0;
    DR_EncoderYawPosition.Z = 0;
    DR_StandStill = false;
    DR_Tilt = false;
    DR_Skid = false;
}


void DR_MotionDetect(void) {
    // 静止检测
    static float standstill = -1;
    if (WHEEL_SPEED_LEFT * WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT * WHEEL_SPEED_RIGHT < 0.001 * 0.001
        && DR_AccChip.X * DR_AccChip.X + DR_AccChip.Y * DR_AccChip.Y + DR_AccChip.Z * DR_AccChip.Z < 0.5 * 0.5
        && GYRO_X * GYRO_X + GYRO_Y * GYRO_Y + GYRO_Z * GYRO_Z < 3 * 0.5 * 0.5) 
    {
        if (standstill < 0) {
            standstill = CURRENT_CLOCK;
        } else if (CURRENT_CLOCK - standstill > 0.5) {
            DR_StandStill = true;
        }
    } else if (standstill >= 0) {
        standstill = -1;
        DR_StandStill = false;
    }
    // 倾斜检测
    if (Attitude_EulerAngle.Pitch * Attitude_EulerAngle.Pitch + Attitude_EulerAngle.Roll * Attitude_EulerAngle.Roll < (5.0f / 180 * PI) * (5.0f / 180 * PI)) {
        if (DR_Tilt) {
            DR_Tilt = false;
        }
    } else if (!DR_Tilt) {
        DR_Tilt = true;
    }
    // 打滑检测
    float errwL = fabs(DR_SpeedwL - DR_SpeedCar.X);
    float errwR = fabs(DR_SpeedwR - DR_SpeedCar.X);
    float errLR = fabs(DR_SpeedLR - DR_SpeedCar.X);
    if (errwL < 0.1 && errwR < 0.1 && errLR < 0.1) 
    {
        if (DR_Skid) {
            DR_Skid = false;
        }
    } else if (!DR_Skid) {
        DR_Skid = true;
    }
}

void DR_Update(void) {
    // 方法1 融合
    float r_yaw, cosyaw, sinyaw, yaw2;
    r_yaw = d / 2 * Attitude_EulerAngleRate.Yaw;
    cosyaw = cosf(-Attitude_EulerAngle.Yaw);
    sinyaw = sinf(-Attitude_EulerAngle.Yaw);
//    yaw2 = Attitude_EulerAngleRate.Yaw * Attitude_EulerAngleRate.Yaw;
//    // 转换为m/s^2, 并减去重力加速度
//    DR_AccChip.X = -ACC_X * GRAVITY;
//    DR_AccChip.Y = -ACC_Y * GRAVITY;
//    DR_AccChip.Z = -ACC_Z * GRAVITY;
//    // 并减去重力加速度
//    VectorTypedef g; g.Z = GRAVITY; g.X = 0; g.Y = 0;
//    Quaternion_Map(&Attitude_Quaternion, &(g.X), &(g.Y), &(g.Z));
//    DR_AccChip.X += g.X;
//    DR_AccChip.Y += g.Y;
//    DR_AccChip.Z += g.Z;
//    
//    // 转换到轴中点(叠加向心加速度+角加速度)
//    DR_AccCar.X = DR_AccChip.X + yaw2 * rmsinm - Attitude_EulerAngleRateDelta.Yaw * rmcosm;
//    DR_AccCar.Y = DR_AccChip.Y - yaw2 * rmcosm - Attitude_EulerAngleRateDelta.Yaw * rmsinm;
//    DR_AccCar.Z = 0;
////    DR_AccCar.X = DR_AccChip.X;
////    DR_AccCar.Y = DR_AccChip.Y;
////    DR_AccCar.Z = 0;
//    // 转换到NED坐标
//    DR_Acc.X = DR_AccCar.X * cosyaw + DR_AccCar.Y * sinyaw; 
//    DR_Acc.Y = -DR_AccCar.X * sinyaw + DR_AccCar.Y * cosyaw; 
//    DR_Acc.Z = DR_AccCar.Z;
//    // 对速度进行积分
//    DR_Speed.X += DR_Acc.X * PERIOD;
//    DR_Speed.Y += DR_Acc.Y * PERIOD;
//    DR_Speed.Z += DR_Acc.Z * PERIOD;
//    // 速度转换回载体坐标系
//    DR_SpeedCar.X = DR_Speed.X * cosyaw - DR_Speed.Y * sinyaw;
//    DR_SpeedCar.Y = DR_Speed.X * sinyaw + DR_Speed.Y * cosyaw;
//    DR_SpeedCar.Z = DR_Speed.Z;

//    // 计算编码器陀螺仪数据
//    DR_SpeedwL = WHEEL_SPEED_LEFT + r_yaw;
//    DR_SpeedwR = WHEEL_SPEED_LEFT - r_yaw;
//    DR_SpeedLR = (WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT) / 2;
//    DR_YawRateLR = (WHEEL_SPEED_LEFT - WHEEL_SPEED_RIGHT) / d;
//    // 选择误差最小项
//    float errwL = DR_SpeedwL - DR_SpeedCar.X;
//    float errwR = DR_SpeedwR - DR_SpeedCar.X;
//    float errLR = DR_SpeedLR - DR_SpeedCar.X;
//    float err;
////    if (fabs(errwL) <= fabs(errwR) && fabs(errwL) <= fabs(errLR)) {
////        err = errwL;
////    } else if (fabs(errwR) <= fabs(errwL) && fabs(errwR) <= fabs(errLR)) {
////        err = errwR;
////    } else if (fabs(errLR) <= fabs(errwL) && fabs(errLR) <= fabs(errwR)) {
//        err = errLR;
////    }
//    // 前进方向限幅校正
//    float limit = Calibration_Limit * PERIOD;
//    err = CONSTRAIT(err, -limit, limit);
//    DR_SpeedCar.X += err;
//    // 侧滑方向限幅校正
//    err = 0 - DR_SpeedCar.Y;
//    err = CONSTRAIT(err, -limit, limit);
//    DR_SpeedCar.Y += err;
//    
//    DR_Speed.X = DR_SpeedCar.X * cosyaw + DR_SpeedCar.Y * sinyaw; 
//    DR_Speed.Y = -DR_SpeedCar.X * sinyaw + DR_SpeedCar.Y * cosyaw; 
//    DR_Speed.Z = DR_SpeedCar.Z;
//    
//    // 对位置进行积分
//    DR_Position.X += DR_Speed.X * PERIOD;
//    DR_Position.Y += DR_Speed.Y * PERIOD;
//    DR_Position.Z += DR_Speed.Z * PERIOD;
    
    // 方法二 仅使用编码器
    // 编码器积分航向角
//    DR_EncoderYaw += DR_YawRateLR / PI * 180 * WHEEL_SPEED_PERIOD;
//    if (DR_EncoderYaw < -180) {
//        DR_EncoderYaw += 360;
//    } else if (DR_EncoderYaw > 180) {
//        DR_EncoderYaw -= 360;
//    }
    // 将航向角上的编码器测得车速转换到NED坐标
    VectorTypedef v;
    v.X = (WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT) / 2 * cosf(DR_EncoderYaw / 180 * PI);
    v.Y = (WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT) / 2 * sinf(DR_EncoderYaw / 180 * PI);
    // 对位置进行积分
    DR_EncoderPosition.X += v.X * PERIOD;
    DR_EncoderPosition.Y += v.Y * PERIOD;
    
    // 方法三 编码器+航向角
    v.X = (WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT) / 2 * cosyaw;
    v.Y = (WHEEL_SPEED_LEFT + WHEEL_SPEED_RIGHT) / 2 * -sinyaw;
    // 对位置进行积分
    DR_EncoderYawPosition.X += v.X * PERIOD;
    DR_EncoderYawPosition.Y += v.Y * PERIOD;
    
    DR_MotionDetect();
}
