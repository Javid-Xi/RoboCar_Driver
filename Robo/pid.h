#ifndef __PID_H
#define __PID_H

#include "sys.h"


#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限


int16_t Motor_PidCtl_A(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_B(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_C(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_D(int16_t spd_target, int16_t spd_current);   //PID控制


#endif

