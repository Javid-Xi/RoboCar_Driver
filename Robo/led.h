/**
************************************************************
* @file         led.h
* @brief        led����
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED_G_Off()			GPIO_SetBits(GPIOA, GPIO_Pin_8)      //LEDG��ɫϨ��
#define LED_G_On()			GPIO_ResetBits(GPIOA, GPIO_Pin_8)    //LEDG��ɫ����
#define LED_G_Toggle()		GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)))	//LEDG��ɫ״̬��ת

void LED_Init(void);

#endif 

