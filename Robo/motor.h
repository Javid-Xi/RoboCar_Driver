/**
************************************************************
* @file         motor.h
* @brief        电机控制
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void MOTOR_Init(uint8_t freq_khz); //电机PWM控制初始化
void MOTOR_A_SetSpeed(int16_t speed);   //电机A控制
void MOTOR_B_SetSpeed(int16_t speed);   //电机B控制
void MOTOR_C_SetSpeed(int16_t speed);   //电机C控制
void MOTOR_D_SetSpeed(int16_t speed);   //电机D控制

#endif

