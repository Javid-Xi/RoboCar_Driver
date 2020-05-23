/**
************************************************************
* @file         pid.c
* @brief        pid控制
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include "pid.h"

pid motor_pid = {130,0,30};
pid yaw_pid = {0,0,0};

/*************************************************
* Function: Yaw_PidCtl
* Description: 	电机A PID控制函数
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Yaw_PidCtl(int16_t yaw_target, int16_t yaw_current)
{
    static int16_t yaw_speed;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = yaw_target - yaw_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;

    yaw_speed += yaw_pid.p.sv * bias * PID_SCALE + yaw_pid.d.sv * (bias - bias_last) * PID_SCALE + yaw_pid.i.sv * bias_integral * PID_SCALE;

    bias_last = bias;

    return yaw_speed;
}

/*************************************************
* Function: Motor_PidCtl_A
* Description: 	电机A PID控制函数
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_A(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_pid.p.sv * bias * PID_SCALE + motor_pid.d.sv * (bias - bias_last) * PID_SCALE + motor_pid.i.sv * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_B
* Description: 	电机B PID控制函数
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_B(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_pid.p.sv * bias * PID_SCALE + motor_pid.d.sv * (bias - bias_last) * PID_SCALE + motor_pid.i.sv * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_C
* Description: 	电机C PID控制函数
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_C(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_pid.p.sv * bias * PID_SCALE + motor_pid.d.sv * (bias - bias_last) * PID_SCALE + motor_pid.i.sv * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_D
* Description: 	电机D PID控制函数
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_D(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_pid.p.sv * bias * PID_SCALE + motor_pid.d.sv * (bias - bias_last) * PID_SCALE + motor_pid.i.sv * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}
