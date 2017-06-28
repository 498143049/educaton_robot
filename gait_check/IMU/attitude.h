#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdbool.h>

#define PI  ((float)3.14159265358)
    
// 参考坐标系使用北东地(NED)坐标系
// North: X轴, East: Y轴, Down: Z轴

// 载体坐标系使用Tait–Bryan欧拉角
// zyx顺规, Yaw-Pitch-Roll
// Yaw:   偏航角(z  轴) ±pi
// Pitch: 俯仰角(y' 轴) ±pi/2
// Roll:  横滚角(x''轴) ±pi

// 四元数
typedef struct {
    float W;
    float X;
    float Y;
    float Z;
} QuaternionTypedef;

// 欧拉角
typedef struct {
    float Roll;
    float Pitch;
    float Yaw;
} EulerAngleTypdef;

// 使用四元数表示的姿态
extern QuaternionTypedef   Attitude_Quaternion;
extern QuaternionTypedef   Attitude_Quaternion_;
// 使用欧拉角表示的姿态
extern EulerAngleTypdef    Attitude_EulerAngle;
extern EulerAngleTypdef    Attitude_EulerAngle_;
// 使用欧拉角表示的姿态角速度(地理坐标系下的)
extern EulerAngleTypdef   Attitude_EulerAngleRate;
// 使用欧拉角表示的姿态角速度加速度(地理坐标系下的)
extern EulerAngleTypdef   Attitude_EulerAngleRateDelta;
// 使用四元数表示的陀螺仪加速度计算得的姿态
extern QuaternionTypedef   Attitude_ExpectQuaternion;
// 使用欧拉角表示的陀螺仪加速度计算得的姿态
extern EulerAngleTypdef    Attitude_ExpectEulerAngle;
extern EulerAngleTypdef    Attitude_ExpectEulerAngle_;
// 使用四元数表示的两种姿态的误差
extern QuaternionTypedef   Attitude_ErrorQuaternion;
// 使用轴角表示的两种姿态的误差
extern QuaternionTypedef   Attitude_ErrorAxisAngle;
// 两种姿态的误差角度
extern float Attitude_ErrorAngle;
// 陀螺仪预计零偏, 单位: rad/秒
#define ATTITUDE_GYRO_BIAS_LIMIT            ((float)0.1 / 180 * PI)
// 陀螺仪预计零偏, 单位: rad/秒
#define ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE   ((float)2 / 180 * PI)
// 陀螺仪零偏跟踪速度, 单位: 1/秒
#define ATTITUDE_GYRO_BIAS_FOLLOW           ((float)0.01)
// 陀螺仪零偏跟踪速度(刚刚开机时), 单位: 1/秒
#define ATTITUDE_GYRO_BIAS_FOLLOW_RELIABLE  ((float)0.2)
// 加速度计姿态跟踪速度, 单位: 1/秒
#define ATTITUDE_ACC_FOLLOW                 ((float)0.5)
// 加速度计姿态跟踪速度(刚刚开机时), 单位: 1/秒
#define ATTITUDE_ACC_FOLLOW_RELIABLE        ((float)20)

#define ATTITUDE_COMPENSATE_LIMIT   (sqrtf(3) * ATTITUDE_GYRO_BIAS_LIMIT / 1000)


// 传感器数据来源定义
#include "bmx055.h"
extern bmx055_t bmx055;
// 三个传感器XYZ轴相同
// X轴指向物体正右侧
// Y轴指向物体正前方
// Z轴指向物体正上方
#define ACC_X   (bmx055.ax) // 加速度数据, 单位: g
#define ACC_Y   (bmx055.ay) // Z轴向上、重力向下时, 取值为正
#define ACC_Z   (bmx055.az)
#define GYRO_X  (bmx055.gx) // 角速度数据, 单位: °/s
#define GYRO_Y  (bmx055.gy) // 绕轴逆时针(右手螺旋)为正
#define GYRO_Z  (bmx055.gz)
#define PERIOD  (0.01f)     // 角速度采样周期, 单位: s
#define MAG_X   (bmx055.mx) // 磁力计数据, 单位: μT
#define MAG_Y   (bmx055.my) // 磁感线投影在各轴的分量
#define MAG_Z   (bmx055.mz)

extern float vx, vy, vz;
extern float ax, ay, az;
extern float ex, ey, ez;
extern float gx, gy, gz;
extern float exi, eyi, ezi;

void Quaternion_Normalize(QuaternionTypedef *q);
void Quaternion_Invert(QuaternionTypedef *p, const QuaternionTypedef *q);
void Quaternion_Multi(QuaternionTypedef *result, const QuaternionTypedef *p, const QuaternionTypedef *q);
void Quaternion_Rotation(const QuaternionTypedef *q, float *nx, float *ny, float *nz);
void Quaternion_Map(const QuaternionTypedef *q, float *nx, float *ny, float *nz);
void Quaternion_FromAxisAngle(QuaternionTypedef *q, const QuaternionTypedef *a);
void Quaternion_ToAxisAngle(const QuaternionTypedef *q, QuaternionTypedef *a);
void Quaternion_FromEulerAngle(QuaternionTypedef *q, const EulerAngleTypdef *e);
void Quaternion_ToEulerAngle(const QuaternionTypedef *q, EulerAngleTypdef *e);
void Quaternion_FromGyro(QuaternionTypedef *q, float wx, float wy, float wz, float dt);
void Quaternion_UpdateFromGyro(QuaternionTypedef *q, float x, float y, float z, float dt);
bool Quaternion_FromAccMag(QuaternionTypedef *q, float ax, float ay, float az, float mx, float my, float mz);

void Attitude_Init(void);

void Attitude_UpdateGyro(void);

void Attitude_UpdateAccMag(void);

void Attitude_UpdateAcc(void);

void Attitude_MahonyAHRSupdateIMU(void);

#endif
