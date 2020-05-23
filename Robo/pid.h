#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "uart.h"

#define PID_SCALE  0.01f  //PID����ϵ��
#define PID_INTEGRAL_UP 1000  //��������

//pid data
typedef	struct {
    short_union p;
    short_union i;
    short_union d;
} pid;

int16_t Motor_PidCtl_A(int16_t spd_target, int16_t spd_current);   //PID����
int16_t Motor_PidCtl_B(int16_t spd_target, int16_t spd_current);   //PID����
int16_t Motor_PidCtl_C(int16_t spd_target, int16_t spd_current);   //PID����
int16_t Motor_PidCtl_D(int16_t spd_target, int16_t spd_current);   //PID����

void PID_Set(rcv_data* data);
#endif

