/**
************************************************************
* @file         adc.c
* @brief        adc + dma 多通道读取 电量读取
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __DMC_H
#define __DMC_H

#include "sys.h"

void ADC_DMA_Init(void);
uint16_t Get_power(void);

#endif
