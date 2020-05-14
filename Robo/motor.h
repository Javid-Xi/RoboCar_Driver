/**
************************************************************
* @file         motor.h
* @brief        �������
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void MOTOR_Init(uint8_t freq_khz); //���PWM���Ƴ�ʼ��
void MOTOR_A_SetSpeed(int16_t speed);   //���A����
void MOTOR_B_SetSpeed(int16_t speed);   //���B����
void MOTOR_C_SetSpeed(int16_t speed);   //���C����
void MOTOR_D_SetSpeed(int16_t speed);   //���D����

#endif

