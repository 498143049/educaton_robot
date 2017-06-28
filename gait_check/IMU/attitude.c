#include <math.h>
#include <string.h>

#include "attitude.h"
#include "main.h"

//#include "MahonyAHRS.h"

QuaternionTypedef  Attitude_Quaternion;
QuaternionTypedef  Attitude_Quaternion_;
EulerAngleTypdef   Attitude_EulerAngle;
EulerAngleTypdef   Attitude_EulerAngle_;

EulerAngleTypdef   Attitude_EulerAngleRate;
EulerAngleTypdef    Attitude_EulerAngleRateDelta;

QuaternionTypedef  Attitude_ExpectQuaternion;
EulerAngleTypdef   Attitude_ExpectEulerAngle;
EulerAngleTypdef   Attitude_ExpectEulerAngle_;

float Attitude_ErrorAngle;

float Attitude_InitTime = -1;

// 误差补偿阈值对应三角函数值, 用于加速运算
static double ATTITUDE_COMPENSATE_LIMIT_HALF_SIN;
static double ATTITUDE_COMPENSATE_LIMIT_HALF_COS;

#define constrait(x,min,max)    ((x>max)?max:(x<min)?min:x)

// 四元数归一化(单位化)
void Quaternion_Normalize(QuaternionTypedef *q) {
    float s = sqrtf(q->W * q->W + q->X * q->X + q->Y * q->Y + q->Z * q->Z);
    if (isnan(s) || s <= 0) {
        q->W = 1;
        q->X = 0;
        q->Y = 0;
        q->Z = 0;
    } else {
        q->W /= s;
        q->X /= s;
        q->Y /= s;
        q->Z /= s;
    }
}

// 四元数求逆, p=q^-1=q*/(|q|^2), 原始四元数需为单位四元数
void Quaternion_Invert(QuaternionTypedef *p, const QuaternionTypedef *q) {
    p->W = q->W;
    p->X = -q->X;
    p->Y = -q->Y;
    p->Z = -q->Z;
}

// 四元数乘法, 旋转的复合, result=pq
void Quaternion_Multi(QuaternionTypedef *result, const QuaternionTypedef *p, const QuaternionTypedef *q) {
    result->W = p->W * q->W - p->X * q->X - p->Y * q->Y - p->Z * q->Z;
    result->X = p->W * q->X + p->X * q->W + p->Y * q->Z - p->Z * q->Y;
    result->Y = p->W * q->Y - p->X * q->Z + p->Y * q->W + p->Z * q->X;
    result->Z = p->W * q->Z + p->X * q->Y - p->Y * q->X + p->Z * q->W;
}

// 四元数旋转, 使用四元数q将一个向量旋转到指定方向, v = q v q^-1
// q为n系->b系的旋转, 将向量v的参考系从b系变换到n系可以用此函数
void Quaternion_Rotation(const QuaternionTypedef *q, 
    float *nx, float *ny, float *nz) 
{
    QuaternionTypedef qinvert;
    Quaternion_Invert(&qinvert, q);
    QuaternionTypedef n;
    n.W = 0; n.X = *nx; n.Y = *ny; n.Z = *nz;
    QuaternionTypedef tmp;
    Quaternion_Multi(&tmp, q, &n);
    Quaternion_Multi(&n, &tmp, &qinvert);
    *nx = n.X;
    *ny = n.Y;
    *nz = n.Z;
}

// 四元数映射, v = q^-1 v q
// q为n系->b系的旋转, 将向量v的参考系从n系变换到b系可以用此函数
void Quaternion_Map(const QuaternionTypedef *q, 
    float *nx, float *ny, float *nz) 
{
    QuaternionTypedef qinvert;
    Quaternion_Invert(&qinvert, q);
    QuaternionTypedef n;
    n.W = 0; n.X = *nx; n.Y = *ny; n.Z = *nz;
    QuaternionTypedef tmp;
    Quaternion_Multi(&tmp, &qinvert, &n);
    Quaternion_Multi(&n, &tmp, q);
    *nx = n.X;
    *ny = n.Y;
    *nz = n.Z;
}

// 轴角转换为四元数
void Quaternion_FromAxisAngle(QuaternionTypedef *q, const QuaternionTypedef *a) {
    float c = cosf(a->W / 2);
    float s = sinf(a->W / 2);
    q->W = 0;
    q->X = a->X;
    q->Y = a->Y;
    q->Z = a->Z;
    Quaternion_Normalize(q);
    q->W = c;
    q->X *= s;
    q->Y *= s;
    q->Z *= s;
}

// 四元数转换为轴角
void Quaternion_ToAxisAngle(const QuaternionTypedef *q, QuaternionTypedef *a) {
    a->W = 0;
    a->X = q->X;
    a->Y = q->Y;
    a->Z = q->Z;
    Quaternion_Normalize(a);
    a->W = acosf(q->W) * 2;
}

// 欧拉角转换为四元数
void Quaternion_FromEulerAngle(QuaternionTypedef *q, const EulerAngleTypdef *e) {
    float cosx = cosf(e->Roll / 2);
    float sinx = sinf(e->Roll / 2);
    float cosy = cosf(e->Pitch / 2);
    float siny = sinf(e->Pitch / 2);
    float cosz = cosf(e->Yaw / 2);
    float sinz = sinf(e->Yaw / 2);
    q->W = cosx * cosy * cosz + sinx * siny * sinz;
    q->X = sinx * cosy * cosz - cosx * siny * sinz;
    q->Y = cosx * siny * cosz + sinx * cosy * sinz;
    q->Z = cosx * cosy * sinz - sinx * siny * cosz;
}

// 四元数转换为欧拉角
void Quaternion_ToEulerAngle(const QuaternionTypedef *q, EulerAngleTypdef *e) {
    e->Roll = atan2f(2 * (q->W * q->X + q->Y * q->Z), 1 - 2 * (q->X * q->X + q->Y * q->Y));
    float k = 2 * (q->W * q->Y - q->Z * q->X);
    if (k > 1) k = 1;
    else if (k < -1) k = -1;
    e->Pitch = asinf(k);
    e->Yaw = atan2f(2 * (q->W * q->Z + q->X * q->Y), 1 - 2 * (q->Y * q->Y + q->Z * q->Z));
}

// 角速度+周期转换为增量四元数
void Quaternion_FromGyro(QuaternionTypedef *q, float wx, float wy, float wz, float dt) {
    // 好理解的算法
//    EulerAngleTypdef e;
//    e.Roll  = wx * dt;
//    e.Pitch = wy * dt;
//    e.Yaw   = wz * dt;
//    Quaternion_FromEulerAngle(q, &e);
    // 快速算法
    q->W = 1;
    q->X = wx * dt / 2;
    q->Y = wy * dt / 2;
    q->Z = wz * dt / 2;
    Quaternion_Normalize(q);
}

// 使用角速度更新四元数, (经实验验证, 换成double也不会影响解算精度)
void Quaternion_UpdateFromGyro(QuaternionTypedef *q, float x, float y, float z, float dt) {
    // 好理解的算法, 使用角度增量, 不可靠
//    QuaternionTypedef dq;
//    Quaternion_FromGyro(&dq, x, y, z, PERIOD);
//    QuaternionTypedef last;
//    memcpy(&last, &Attitude_Quaternion, sizeof(QuaternionTypedef));
//    Quaternion_Multi(&Attitude_Quaternion, &dq, &last);
    
    // 一阶龙哥库塔法Runge-Kunta, 等价于使用角度增量
    float dW = 0.5 * (-q->X * x - q->Y * y - q->Z * z) * dt;
    float dX = 0.5 * (q->W * x + q->Y * z - q->Z * y) * dt;
    float dY = 0.5 * (q->W * y - q->X * z + q->Z * x) * dt;
    float dZ = 0.5 * (q->W * z + q->X * y - q->Y * x) * dt;
    q->W += dW;
    q->X += dX;
    q->Y += dY;
    q->Z += dZ;
    
    // 二阶毕卡算法, (运算结果和一阶龙哥库塔法, 经实验没差)
//    float halfT = dt / 2;
//    float delta_2d8 = 1- 0.5 * halfT * halfT * (x * x + y * y + z * z);
//    float qw = delta_2d8 * q->W + (-q->X*x - q->Y*y - q->Z*z) * halfT;
//    float qx = delta_2d8 * q->X + ( q->W*x + q->Y*z - q->Z*y) * halfT;
//    float qy = delta_2d8 * q->Y + ( q->W*y - q->X*z + q->Z*x) * halfT;
//    float qz = delta_2d8 * q->Z + ( q->W*z + q->X*y - q->Y*x) * halfT;
//    q->W = qw;
//    q->X = qx;
//    q->Y = qy;
//    q->Z = qz;
    
    Quaternion_Normalize(q);
}

float xN, xE, xD;
float yN, yE, yD;
float zN, zE, zD;
// 使用重力加速度和磁力计数据生成四元数, 成功返回true
// 在acc(0, 0, -1)且mag(x, x, x)情况下不可用, 在acc(0, x, x)且mag(-1, x, x)情况下不可用
bool Quaternion_FromAccMag(QuaternionTypedef *q, float ax, float ay, float az, float mx, float my, float mz) {
    float k;
    // 方向余弦矩阵(DCM)
    // 重力方向为Down轴
    xD = ax; yD = ay; zD = az;
    // 归一化Down轴
    k = sqrtf(xD * xD + yD * yD + zD * zD);
    if (isnan(k) || k <= 0) return false;
    xD /= k; yD /= k; zD /= k; 
    // 磁力反方向为North轴
    xN = -mx; yN = -my; zN = -mz;
    // 正交化North轴
    k = xN * xD + yN * yD + zN * zD;
    xN -= xD * k;
    yN -= yD * k;
    zN -= zD * k;
    // 归一化North轴
    k = sqrtf(xN * xN + yN * yN + zN * zN);
    if (isnan(k) || k <= 0) return false;
    xN /= k; yN /= k; zN /= k; 
    // 求East轴
    xE = yD * zN - zD * yN;
    yE = zD * xN - xD * zN;
    zE = xD * yN - yD * xN;
    // 归一化East轴
    k = sqrtf(xE * xE + yE * yE + zE * zE);
    if (isnan(k) || k <= 0) return false;
    xE /= k; yE /= k; zE /= k; 
    
    // 旋转矩阵(方向余弦矩阵)转换为四元数
    q->W = 0.5 * sqrtf(xN + yE + zD + 1);
    if (isnan(q->W) || q->W <= 0) return false;
    q->X = (yD - zE) / (4 * q->W);
    q->Y = (zN - xD) / (4 * q->W);
    q->Z = (xE - yN) / (4 * q->W);
    Quaternion_Normalize(q);
    return true;
}

// 计算四元数p到q的转换四元数error, p error = q
void Quaternion_GetError(QuaternionTypedef *error, const QuaternionTypedef *p, const QuaternionTypedef *q) {
    QuaternionTypedef p_invert;
    Quaternion_Invert(&p_invert, p);
    Quaternion_Multi(error, &p_invert, q);
}

// p = p q, 同时限制q的转角
void Quaternion_CompensateLimited(QuaternionTypedef *p, const QuaternionTypedef *q) {
    // FIXME
    // 限制校正阈值, 即限制误差的轴角的角度大小
    // 直接限制w=cos(θ/2)的值, 并调整相应v(x,y,z)值, 仅在程序开始时需要进行一次三角函数运算
    QuaternionTypedef compensate;
    if (q->W < ATTITUDE_COMPENSATE_LIMIT_HALF_COS) {
        float k = 1 / sqrtf(1 - q->W * q->W);
        k *= ATTITUDE_COMPENSATE_LIMIT_HALF_SIN;
        compensate.W = ATTITUDE_COMPENSATE_LIMIT_HALF_COS;
        compensate.X = q->X * k;
        compensate.Y = q->Y * k;
        compensate.Z = q->Z * k;
    } else {
        memcpy(&compensate, q, sizeof(QuaternionTypedef));
    }
    // 执行校正
    QuaternionTypedef old;
    memcpy(&old, p, sizeof(QuaternionTypedef));
    Quaternion_Multi(p, &old, &compensate);
}


// 姿态初始化
void Attitude_Init(void) {
    // 计算常量
    ATTITUDE_COMPENSATE_LIMIT_HALF_SIN = sin(ATTITUDE_COMPENSATE_LIMIT / 2);
    ATTITUDE_COMPENSATE_LIMIT_HALF_COS = cos(ATTITUDE_COMPENSATE_LIMIT / 2);
    
//    if (MAG_X != 0 || 
//        MAG_Y != 0 || 
//        MAG_Z != 0) 
//    { // 磁力计和加速度计有数据
//        Quaternion_FromAccMag(&Attitude_Quaternion, 
//            ACC_X, ACC_Y, ACC_Z, 
//            MAG_X, MAG_Y, MAG_Z
//        );
//    } else 
    if (ACC_X != 0 || 
        ACC_Y != 0 || 
        ACC_Z != 0) 
    { // 加速度计有数据, 默认开机X轴指向正北
        Quaternion_FromAccMag(&Attitude_Quaternion, 
            ACC_X, ACC_Y, ACC_Z, 
            -1, 0, 0
        );
    } else { // 无数据, 默认开机欧拉角为0
        Attitude_Quaternion.W = 1;
        Attitude_Quaternion.X = 0;
        Attitude_Quaternion.Y = 0;
        Attitude_Quaternion.Z = 0;
    }
    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
//    memcpy(&Attitude_Quaternion_, &Attitude_Quaternion, sizeof(QuaternionTypedef));
//    memcpy(&Attitude_EulerAngle_, &Attitude_EulerAngle, sizeof(EulerAngleTypdef));
//    Quaternion_ToAxisAngle(&Attitude_Quaternion, &Attitude_AxisAngle);
    
    
    Attitude_InitTime = MY_GetClock();
}

// 快速姿态更新, 仅使用陀螺仪数据
void Attitude_UpdateGyro(void) {
    // 保留上一次的欧拉角和四元数
    EulerAngleTypdef last_eulerangle;
    memcpy(&last_eulerangle, &Attitude_EulerAngle, sizeof(EulerAngleTypdef));
    QuaternionTypedef last_quanternion;
    memcpy(&last_quanternion, &Attitude_Quaternion, sizeof(QuaternionTypedef));
    // 进行姿态更新
    float gx = GYRO_X / 180 * PI;
    float gy = GYRO_Y / 180 * PI;
    float gz = GYRO_Z / 180 * PI;
    
    // 校正安装误差
    //Quaternion_Map(&Install_ErrorQuat, &gx, &gy, &gz);
    
    Quaternion_UpdateFromGyro(&Attitude_Quaternion, gx, gy, gz, PERIOD);
    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
//    Quaternion_UpdateFromGyro2(&Attitude_Quaternion_, gx, gy, gz, PERIOD);
//    Quaternion_ToEulerAngle(&Attitude_Quaternion_, &Attitude_EulerAngle_);
    
    // 计算欧拉角速度
    // Yaw为偏航角速度, 为绕NED中的D轴(Z轴)旋转的角速度, 使用四元数计算
//    QuaternionTypedef g, tmp;
//    g.W = 0; g.X = gx; g.Y = gy; g.Z = gz;
//    Quaternion_Invert(&last_quanternion, &last_quanternion);
//    Quaternion_Multi(&tmp, &last_quanternion, &g);
//    Quaternion_Invert(&last_quanternion, &last_quanternion);
//    Quaternion_Multi(&g, &tmp, &last_quanternion);
//    Attitude_EulerAngleRateDelta.Yaw = (g.Z - Attitude_EulerAngleRate.Yaw) / PERIOD;
//    Attitude_EulerAngleRate.Yaw  = g.Z;
    Attitude_EulerAngleRateDelta.Yaw = (gz - Attitude_EulerAngleRate.Yaw) / PERIOD;
    Attitude_EulerAngleRate.Yaw  = gz;
//    // Pitch为俯仰角速度, 为绕Y'轴旋转的角速度, 使用???计算
//    if (fabs(last_eulerangle.Pitch - Attitude_EulerAngle.Pitch) < PI / 2) {
//        Attitude_EulerAngleRate.Pitch = Attitude_EulerAngle.Pitch - last_eulerangle.Pitch;
//    } else if (Attitude_EulerAngle.Pitch - last_eulerangle.Pitch > PI / 2) {
//        Attitude_EulerAngleRate.Pitch = -PI + (Attitude_EulerAngle.Pitch - last_eulerangle.Pitch);
//    } else if (Attitude_EulerAngle.Pitch - last_eulerangle.Pitch < -PI / 2) {
//        Attitude_EulerAngleRate.Pitch = PI + (Attitude_EulerAngle.Pitch - last_eulerangle.Pitch);
//    }
////    Attitude_EulerAngleRate.Pitch /= PERIOD;
//    // Roll为横滚角速度, 绕X''轴旋转的角速度, 直接使用陀螺仪数据
//    Attitude_EulerAngleRateDelta.Roll = (gx - Attitude_EulerAngleRate.Roll) / PERIOD;
//    Attitude_EulerAngleRate.Roll  = gx;
}

float vx, vy, vz;
float ax, ay, az;

float mx, my, mz;
float hx, hy, bx, bz;
float wx, wy, wz;

float ex, ey, ez;
float gx, gy, gz;
float exi = 0.0f / 180 * PI, eyi = 0.0f / 180 * PI, ezi = 0;

// 深度姿态融合, 使用加速度计和磁力计校正
void Attitude_UpdateAccMag(void) {
    float k;
    float sum_acc = sqrtf(ACC_X * ACC_X + ACC_Y * ACC_Y + ACC_Z * ACC_Z);
    // 方法一
//    // 计算加速度计和磁力计测得姿态, 如果出错则返回
//    if (!Quaternion_FromAccMag(&Attitude_ExpectQuaternion, 
//        ACC_X, ACC_Y, ACC_Z, 
//        MAG_X, MAG_Y, MAG_Z))
//        return;
//    // 计算估计姿态到测得姿态之间的误差
//    QuaternionTypedef Attitude_ErrorQuaternion;
//    Quaternion_GetError(&Attitude_ErrorQuaternion, &Attitude_Quaternion, &Attitude_ExpectQuaternion);
//    Attitude_ErrorAngle = acosf(Attitude_ErrorQuaternion.W) * 2;
//    // 执行校正
//    Quaternion_CompensateLimited(&Attitude_Quaternion, &Attitude_ErrorQuaternion);
    
    // 方法二 Mahony算法, 互补滤波
#define q0  (Attitude_Quaternion.W)
#define q1  (Attitude_Quaternion.X)
#define q2  (Attitude_Quaternion.Y)
#define q3  (Attitude_Quaternion.Z)
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
    
    float Kp, Ki; 
    // ② 将b系实测重力加速度a单位化
    ax = ACC_X; ay = ACC_Y; az = ACC_Z;
    k = sqrtf(ax * ax + ay * ay + az * az);
    if (isnan(k) || k <= 0) return;
    ax /= k; ay /= k; az /= k;
    
    // ① 将n系D轴(1,0,0)重力加速度n转换到b系, 作为期望(理想)重力方向
//    vx = 2 * (Attitude_Quaternion.X * Attitude_Quaternion.Z - 
//        Attitude_Quaternion.W * Attitude_Quaternion.Y);  
//    vy = 2 * (Attitude_Quaternion.W * Attitude_Quaternion.X + 
//        Attitude_Quaternion.Y * Attitude_Quaternion.Z);  
//    vz = Attitude_Quaternion.W * Attitude_Quaternion.W - 
//        Attitude_Quaternion.X * Attitude_Quaternion.X - 
//        Attitude_Quaternion.Y * Attitude_Quaternion.Y + 
//        Attitude_Quaternion.Z * Attitude_Quaternion.Z;  
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = 2 * (q0q0 - 0.5f + q3q3);
        
    // 将b系实测磁力m单位化
    mx = -MAG_X; my = -MAG_Y; mz = -MAG_Z;
    k = sqrtf(mx * mx + my * my + mz * mz);
    if (isnan(k) || k <= 0) return;
    mx /= k; my /= k; mz /= k;
    
    // 将实测磁力m转换到n系, 并分解为Z轴分量bz与xOy平面分量bx
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
    // 使用前一时刻姿态将实测磁力的分量b=(bx, 0, bz), 转换到b系作为期望(理想)磁力方向
    wx = 2 * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    wy = 2 * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    wz = 2 * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));
    
    // ③ 求e=a×v=n·sin(θ), 使用叉乘, 计算二者偏差e
    ex = ay * vz - az * vy + my * wz - mz * wy;
    ey = az * vx - ax * vz + mz * wx - mx * wz;
    ez = ax * vy - ay * vx + mx * wy - my * wx;
    // ④ 使用PI控制器
    // 积分分离, 在角度偏差较大时, 停用积分, 但是积分项依然生效
    if (sqrtf(ex * ex + ey * ey + ez * ez) / PI * 180 < 1) {
        Ki = ATTITUDE_GYRO_BIAS_FOLLOW; 
        exi += ex * Ki / 1000;
        eyi += ey * Ki / 1000;
        ezi += ez * Ki / 1000;
        exi = constrait(exi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        eyi = constrait(eyi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        ezi = constrait(ezi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
    }
    // 在加速度和比较离谱时, 停用比例项, 但是积分项依然生效
    if (sum_acc < 1.02 && sum_acc > 0.98) {
        Kp = ATTITUDE_ACC_FOLLOW_RELIABLE;
        gx = constrait(Kp * ex + exi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
        gy = constrait(Kp * ey + eyi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
        gz = constrait(Kp * ez + ezi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
    } else {
        Kp = ATTITUDE_ACC_FOLLOW;
        gx = constrait(Kp * ex + exi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        gy = constrait(Kp * ey + eyi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        gz = constrait(Kp * ez + ezi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
    }
    // ⑤ 更新四元数
    Quaternion_UpdateFromGyro(&Attitude_Quaternion, gx, gy, gz, 0.001);
    
    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
}

// 深度姿态融合, 使用加速度计校正
void Attitude_UpdateAcc(void) {
    float k;
    float sum_acc = sqrtf(ACC_X * ACC_X + ACC_Y * ACC_Y + ACC_Z * ACC_Z);
    
    // 方法一 Z轴方向的圆锥运动将导致偏航角上的误差累积!!! 不可用!
//    // 为了解决这个问题, 需要搭配一定的自旋, 因此需要使用四元数在偏航角不变的约束下进行旋转, 比较麻烦
//    // ① 将n系D轴(1,0,0)重力加速度n转换到b系
//    vx = 2 * (Attitude_Quaternion.X * Attitude_Quaternion.Z - 
//        Attitude_Quaternion.W * Attitude_Quaternion.Y);  
//    vy = 2 * (Attitude_Quaternion.W * Attitude_Quaternion.X + 
//        Attitude_Quaternion.Y * Attitude_Quaternion.Z);  
//    vz = Attitude_Quaternion.W * Attitude_Quaternion.W - 
//        Attitude_Quaternion.X * Attitude_Quaternion.X - 
//        Attitude_Quaternion.Y * Attitude_Quaternion.Y + 
//        Attitude_Quaternion.Z * Attitude_Quaternion.Z;  
//    // ② 将b系实测重力加速度a单位化
//    ax = ACC_X; ay = ACC_Y; az = ACC_Z;
//    float k = sqrtf(ax * ax + ay * ay + az * az);
//    if (isnan(k) || k <= 0) return;
//    ax /= k; ay /= k; az /= k;
//    // ③ 求e=a×v=n·sin(θ), 使用叉乘, 计算二者偏差e
//    ex = ay * vz - az * vy;
//    ey = az * vx - ax * vz;
//    ez = ax * vy - ay * vx;
//    k = sqrtf(ex * ex + ey * ey + ez * ez);
//    if (isnan(k) || k <= 0) return;
//    ex /= k; ey /= k; ez /= k;
//    float ew = acosf(sqrtf(1 - k * k));
//    // ⑥ 根据转角w和转轴v计算增量四元数e
//    QuaternionTypedef e;
//    e.W = cosf(ew / 2);
//    e.X = ex * sinf(ew / 2); e.Y = ey * sinf(ew / 2); e.Z = ez * sinf(ew / 2);
//    // ④ 执行校正
//    Quaternion_CompensateLimited(&Attitude_Quaternion, &e);
//    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
    
    // 方法二
//    // ① 使偏航角为0, 并使用加速度数据计算, 得到姿态四元数p
//    QuaternionTypedef p;
//    float k;
//    // 方向余弦矩阵(DCM)
//    // 重力方向为Down轴
//    xD = ACC_X; yD = ACC_Y; zD = ACC_Z;
//    // 归一化Down轴
//    k = sqrtf(xD * xD + yD * yD + zD * zD);
//    if (isnan(k) || k <= 0) return;
//    xD /= k; yD /= k; zD /= k; 
//    // 磁力反方向为North轴
//    // 正交化North轴
//    xN = 1 - xD * xD;
//    yN = -yD * xD;
//    zN = -zD * xD;
//    // 归一化North轴
//    k = sqrtf(xN * xN + yN * yN + zN * zN);
//    if (isnan(k) || k <= 0) return;
//    xN /= k; yN /= k; zN /= k; 
//    // 求East轴
//    xE = yD * zN - zD * yN;
//    yE = zD * xN - xD * zN;
//    zE = xD * yN - yD * xN;
//    // 旋转矩阵(方向余弦矩阵)转换为四元数
//    p.W = 0.5 * sqrtf(xN + yE + zD + 1);
//    if (isnan(p.W) || p.W <= 0) return;
//    p.X = (yD - zE) / (4 * p.W);
//    p.Y = (zN - xD) / (4 * p.W);
//    p.Z = (xE - yN) / (4 * p.W);
//    // ② 令偏航角=上一次的偏航角, 使用四元数乘法将偏航角生成的四元数q合成到p中, 得到估计姿态四元数
//    QuaternionTypedef q;
//    q.W = cosf(Attitude_EulerAngle.Yaw / 2);
//    q.X = 0;
//    q.Y = 0;
//    q.Z = sinf(Attitude_EulerAngle.Yaw / 2);
////    Quaternion_Multi(&Attitude_ExpectQuaternion, &q, &p);
//    Attitude_ExpectQuaternion.W = q.W * p.W - q.Z * p.Z;
//    Attitude_ExpectQuaternion.X = q.W * p.X - q.Z * p.Y;
//    Attitude_ExpectQuaternion.Y = q.W * p.Y + q.Z * p.X;
//    Attitude_ExpectQuaternion.Z = q.W * p.Z + q.Z * p.W;
//    Quaternion_ToEulerAngle(&Attitude_ExpectQuaternion, &Attitude_ExpectEulerAngle);
//    // ③ 计算估计姿态Attitude_Quaternion(由陀螺仪积分)到实测姿态Attitude_ExpectQuaternion(使用加速度计和已知偏航角计算)之间的误差四元数
//    QuaternionTypedef Attitude_ErrorQuaternion;
//    Quaternion_GetError(&Attitude_ErrorQuaternion, &Attitude_Quaternion, &Attitude_ExpectQuaternion);
////    Attitude_ErrorAngle = acosf(Attitude_ErrorQuaternion.W) * 2;
//    // ④ 执行校正
//    Quaternion_CompensateLimited(&Attitude_Quaternion, &Attitude_ErrorQuaternion);
//    Quaternion_Normalize(&Attitude_Quaternion);
//    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
    
    // 方法三 Mahony算法, 互补
    float Kp, Ki; 
    // ① 将n系D轴(1,0,0)重力加速度n转换到b系
    vx = 2 * (Attitude_Quaternion.X * Attitude_Quaternion.Z - 
        Attitude_Quaternion.W * Attitude_Quaternion.Y);  
    vy = 2 * (Attitude_Quaternion.W * Attitude_Quaternion.X + 
        Attitude_Quaternion.Y * Attitude_Quaternion.Z);  
    vz = Attitude_Quaternion.W * Attitude_Quaternion.W - 
        Attitude_Quaternion.X * Attitude_Quaternion.X - 
        Attitude_Quaternion.Y * Attitude_Quaternion.Y + 
        Attitude_Quaternion.Z * Attitude_Quaternion.Z;  
    // ② 将b系实测重力加速度a单位化
    ax = ACC_X; ay = ACC_Y; az = ACC_Z;
    
    // 校正安装误差
//    Quaternion_Map(&Install_ErrorQuat, &ax, &ay, &az);
    
    k = sqrtf(ax * ax + ay * ay + az * az);
    if (isnan(k) || k <= 0) return;
    ax /= k; ay /= k; az /= k;
    // ③ 求e=a×v=n·sin(θ), 使用叉乘, 计算二者偏差e
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    // ④ 使用PI控制器
    // 积分分离, 在角度偏差较大时, 停用积分, 但是积分项依然生效
    if (sqrtf(ex * ex + ey * ey + ez * ez) / PI * 180 < 1) {
        Ki = ATTITUDE_GYRO_BIAS_FOLLOW; 
        exi += ex * Ki * PERIOD;
        eyi += ey * Ki * PERIOD;
        ezi += ez * Ki * PERIOD;
        exi = constrait(exi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        eyi = constrait(eyi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
        ezi = constrait(ezi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
    }
    // 在加速度和比较离谱时, 停用比例项, 但是积分项依然生效
//    if (sum_acc < 1.02 && sum_acc > 0.98) {
        Kp = ATTITUDE_ACC_FOLLOW_RELIABLE;
        gx = constrait(Kp * ex + exi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
        gy = constrait(Kp * ey + eyi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
        gz = constrait(Kp * ez + ezi, -ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE, ATTITUDE_GYRO_BIAS_LIMIT_RELIABLE);
//    } else {
//        Kp = ATTITUDE_ACC_FOLLOW;
//        gx = constrait(Kp * ex + exi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
//        gy = constrait(Kp * ey + eyi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
//        gz = constrait(Kp * ez + ezi, -ATTITUDE_GYRO_BIAS_LIMIT, ATTITUDE_GYRO_BIAS_LIMIT);
//    }
    // ⑤ 更新四元数
    Quaternion_UpdateFromGyro(&Attitude_Quaternion, gx, gy, gz, 0.001);
    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);

}

//void Attitude_MahonyAHRSupdateIMU(void) {
//    q0 = Attitude_Quaternion.W;
//    q1 = Attitude_Quaternion.X;
//    q2 = Attitude_Quaternion.Y;
//    q3 = Attitude_Quaternion.Z;
//    MahonyAHRSupdateIMU(GYRO_X / 180 * PI, GYRO_Y / 180 * PI, GYRO_Z / 180 * PI, ACC_X, ACC_Y, ACC_Z, PERIOD);
//    Attitude_Quaternion.W = q0;
//    Attitude_Quaternion.X = q1;
//    Attitude_Quaternion.Y = q2;
//    Attitude_Quaternion.Z = q3;
//    Quaternion_ToEulerAngle(&Attitude_Quaternion, &Attitude_EulerAngle);
//}
