#ifndef __CONTROL_H
#define __CONTROL_H

extern float Control_DesiredSpeed;    // 期望速度
extern float Control_DesiredDiff;     // 期望差速

void Control_Init(void);

void Control_Main(void);

#endif
