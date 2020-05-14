/**
************************************************************
* @file         tim.h
* @brief        ��ʱ������
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __TIM_H
#define __TIM_H

#include "stm32f10x.h"

void TIM6_Init(uint16_t cnt_us);  //TIM6��ʼ��
void TIM6_Cmd(FunctionalState NewState); //TIM6��ʱ�������ر�
uint8_t TIM_CheckIrqStatus(void);//��־λȷ��

#endif
