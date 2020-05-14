/**
************************************************************
* @file         tim.h
* @brief        定时器驱动
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __TIM_H
#define __TIM_H

#include "stm32f10x.h"

void TIM6_Init(uint16_t cnt_us);  //TIM6初始化
void TIM6_Cmd(FunctionalState NewState); //TIM6定时器开启关闭
uint8_t TIM_CheckIrqStatus(void);//标志位确认

#endif
