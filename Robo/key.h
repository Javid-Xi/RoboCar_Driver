/**
************************************************************
* @file         key.c
* @brief        按键中断驱动 控制底盘控制模式
* @author       Javid
* @date         2020-01-20
* @version      1.0
*
***********************************************************/

#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

void KEY_Init(void);
uint8_t Mode_get(void);

#endif

