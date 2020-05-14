#include "pid.h"


int16_t motor_kp = 130; //PID参数
int16_t motor_ki = 0; //PID参数
int16_t motor_kd = 30; //PID参数


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

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

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

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

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

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

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

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}
