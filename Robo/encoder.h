/**
************************************************************
* @file         encoder.h
* @brief        ��ʱ��������ģʽ����
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void ENCODER_AB_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_AB_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_AB_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_CD_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_CD_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_CD_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_EF_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_EF_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_EF_SetCounter(uint16_t count);    //���������ü�������ֵ

void ENCODER_GH_Init(uint16_t cycle);          //��������ʼ��
uint16_t ENCODER_GH_GetCounter(void);          //��������ȡ��������ֵ
void ENCODER_GH_SetCounter(uint16_t count);    //���������ü�������ֵ

#endif

