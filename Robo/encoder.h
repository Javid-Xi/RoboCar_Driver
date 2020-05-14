/**
************************************************************
* @file         encoder.h
* @brief        定时器编码器模式驱动
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void ENCODER_AB_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_AB_GetCounter(void);          //编码器获取计数器数值
void ENCODER_AB_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_CD_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_CD_GetCounter(void);          //编码器获取计数器数值
void ENCODER_CD_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_EF_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_EF_GetCounter(void);          //编码器获取计数器数值
void ENCODER_EF_SetCounter(uint16_t count);    //编码器设置计数器数值

void ENCODER_GH_Init(uint16_t cycle);          //编码器初始化
uint16_t ENCODER_GH_GetCounter(void);          //编码器获取计数器数值
void ENCODER_GH_SetCounter(uint16_t count);    //编码器设置计数器数值

#endif

