/**
************************************************************
* @file         led.h
* @brief        led驱动
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED_G_Off()			GPIO_SetBits(GPIOA, GPIO_Pin_8)      //LEDG绿色熄灭
#define LED_G_On()			GPIO_ResetBits(GPIOA, GPIO_Pin_8)    //LEDG绿色点亮
#define LED_G_Toggle()		GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)))	//LEDG绿色状态翻转

#define LED_PIN			GPIO_Pin_8
#define LED_PIN_Port 	GPIOA

#define BEEP_PIN		GPIO_Pin_9
#define BEEP_PIN_Port	GPIOB

void LED_Init(void);
void Sysinit_Complete(void);
	
#endif 

