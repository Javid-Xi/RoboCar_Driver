/**
************************************************************
* @file         delay.h
* @brief        ÑÓÊ±º¯Êý
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#ifndef __DELAY_H
#define __DELAY_H
	 
#include "sys.h"

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif 

