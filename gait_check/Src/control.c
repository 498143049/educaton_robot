#include "Control.h"
#include "Motor.h"
#include "Encoder.h"
#include "SEGGER_RTT.h"
#include <stdint.h>

#define CONSTRAIT(x,min,max)    ((x>max)?max:(x<min)?min:x)
#define SET_DUTY_LEFT(x)        (Motor_SetDutyLeft(x))
#define SET_DUTY_RIGHT(x)       (Motor_SetDutyRight(x))
#define SET_DUTY_BOTH(x)        {Motor_SetDutyLeft(x); Motor_SetDutyRight(x);}
#define GET_SPEED_LEFT()        (Encoder_SpeedLeft)
#define GET_SPEED_RIGHT()       (Encoder_SpeedRight)
#define GET_SPEED_AVG()         ((Encoder_SpeedLeft + Encoder_SpeedRight) / 2)

#define IntMax (1.0f)
#define Ksp         (10.0f)
#define Ksi         (4.0f)
#define Ksd         (0.0f)

float Control_DesiredSpeed;    // 期望速度
float Control_DesiredDiff;     // 期望差速

static float IntErrorLeft;
static float IntErrorRight;
static float LastErrorLeft;
static float LastErrorRight;

void Control_Init() {
    Control_DesiredSpeed = 0;
    Control_DesiredDiff = 0;
    IntErrorLeft = 0;
    IntErrorRight = 0;
    LastErrorLeft = 0;
    LastErrorRight = 0;
    SET_DUTY_BOTH(0);
}

void Control_Main() {
    /* 左电机 */
    float left_e = Control_DesiredSpeed - Control_DesiredDiff / 2 - GET_SPEED_LEFT();
    
    float left_p = left_e * Ksp;
    
    IntErrorLeft += left_e * Ksi;
    IntErrorLeft = CONSTRAIT(IntErrorLeft, -IntMax, IntMax);
    if (Control_DesiredSpeed == 0 && left_e == 0) {
        IntErrorLeft = 0;
    }
    float left_i = IntErrorLeft;
    
    float left_d = (left_e - LastErrorLeft) * Ksd;
    LastErrorLeft = left_e;
    
    float left_output = left_p + left_i + left_d;
    /* 右电机 */
    float right_e = Control_DesiredSpeed + Control_DesiredDiff / 2 - GET_SPEED_RIGHT();
    
    float right_p = right_e * Ksp;
    
    IntErrorRight += right_e * Ksi;
    IntErrorRight = CONSTRAIT(IntErrorRight, -IntMax, IntMax);
    if (Control_DesiredSpeed == 0 && right_e == 0) {
        IntErrorRight = 0;
    }
    float right_i = IntErrorRight;
    
    float right_d = (right_e - LastErrorRight) * Ksd;
    LastErrorRight = right_e;
    
    float right_output = right_p + right_i + right_d;
    
    
    SET_DUTY_LEFT(left_output);
    SET_DUTY_RIGHT(right_output);
    
//    char text[100];
//    sprintf(text, "L=%.2f(%.2f) R=%.2f(%.2f)\r\n", Encoder_SpeedLeft, left_output, Encoder_SpeedRight, right_output);
//    SEGGER_RTT_WriteString(0, text);
}
