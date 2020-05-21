/**
************************************************************
* @file         pid.c
* @brief        pid����
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include "pid.h"

int16_t motor_kp = 130; //PID����
int16_t motor_ki = 0; //PID����
int16_t motor_kd = 30; //PID����

int16_t yaw_kp = 130; //PID����
int16_t yaw_ki = 0; //PID����
int16_t yaw_kd = 30; //PID����

/*************************************************
* Function: Yaw_PidCtl
* Description: 	���A PID���ƺ���
* Parameter:  	spd_target:�������ٶ�Ŀ��ֵ
*				spd_current: �������ٶȵ�ǰֵ
* Return: ���PWM�ٶ�
*************************************************/
int16_t Yaw_PidCtl(int16_t yaw_target, int16_t yaw_current)
{
    static int16_t yaw_speed;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = yaw_target - yaw_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;

    yaw_speed += yaw_kp * bias * PID_SCALE + yaw_kd * (bias - bias_last) * PID_SCALE + yaw_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return yaw_speed;
}

/*************************************************
* Function: Motor_PidCtl_A
* Description: 	���A PID���ƺ���
* Parameter:  	spd_target:�������ٶ�Ŀ��ֵ
*				spd_current: �������ٶȵ�ǰֵ
* Return: ���PWM�ٶ�
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
* Description: 	���B PID���ƺ���
* Parameter:  	spd_target:�������ٶ�Ŀ��ֵ
*				spd_current: �������ٶȵ�ǰֵ
* Return: ���PWM�ٶ�
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
* Description: 	���C PID���ƺ���
* Parameter:  	spd_target:�������ٶ�Ŀ��ֵ
*				spd_current: �������ٶȵ�ǰֵ
* Return: ���PWM�ٶ�
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
* Description: 	���D PID���ƺ���
* Parameter:  	spd_target:�������ٶ�Ŀ��ֵ
*				spd_current: �������ٶȵ�ǰֵ
* Return: ���PWM�ٶ�
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

/*************************************************
* Function: PID_Set
* Description: 	PID��������
* Parameter:  rcv_data* data
* Return: none
*************************************************/
void PID_Set(rcv_data* data)
{
    motor_kp = data->p.sv;
    motor_ki = data->i.sv;
    motor_kd = data->d.sv;
}
