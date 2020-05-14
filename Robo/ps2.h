/**
****************************************************************************
* @file         ps2.c
* @brief        ps2ң��������
* @author       Javid
* @date         2020-05-14
* @version      1.0
* @Note			PS2���ݶ���
*   			BYTE   DATA   ����
*  				01     idle
*   			02     0x73   �ֱ��Ĺ���ģʽ
*   			03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
*   			04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
*   			05     data   L2     R2     L1    R1   /\     O     X    ��
*   			06     data   �ұ�ҡ��  0x00 = ��    0xff = ��
*   			07     data   �ұ�ҡ��  0x00 = ��    0xff = ��
*   			08     data   ���ҡ��  0x00 = ��    0xff = ��
*   			09     data   ���ҡ��  0x00 = ��    0xff = ��
***************************************************************************/

#ifndef __PS2_H
#define __PS2_H
 
#include "sys.h"

#define DI   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)           //PB12  ����

#define DO_H  GPIO_SetBits(GPIOB, GPIO_Pin_14)       //����λ��
#define DO_L GPIO_ResetBits(GPIOB, GPIO_Pin_14)      //����λ��

#define CS_H GPIO_SetBits(GPIOB, GPIO_Pin_13)      //CS����
#define CS_L  GPIO_ResetBits(GPIOB, GPIO_Pin_13)      //CS����

#define CLK_H GPIO_SetBits(GPIOB, GPIO_Pin_12)     //ʱ������
#define CLK_L GPIO_ResetBits(GPIOB, GPIO_Pin_12)     //ʱ������

//��������
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2         9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

//#define WHAMMY_BAR		8

#define PSS_RX 5	//��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);			//�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void);
void PS2_Cmd(u8 CMD);
u8 PS2_DataKey(void);			//��ֵ��ȡ
u8 PS2_AnologData(u8 button);	//�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);		//������ݻ�����
#endif





