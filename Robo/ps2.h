/**
****************************************************************************
* @file         ps2.c
* @brief        ps2遥控器驱动
* @author       Javid
* @date         2020-05-14
* @version      1.0
* @Note			PS2数据定义
*   			BYTE   DATA   解释
*  				01     idle
*   			02     0x73   手柄的工作模式
*   			03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
*   			04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
*   			05     data   L2     R2     L1    R1   /\     O     X    口
*   			06     data   右边摇杆  0x00 = 左    0xff = 右
*   			07     data   右边摇杆  0x00 = 上    0xff = 下
*   			08     data   左边摇杆  0x00 = 左    0xff = 右
*   			09     data   左边摇杆  0x00 = 上    0xff = 下
***************************************************************************/

#ifndef __PS2_H
#define __PS2_H
 
#include "sys.h"

#define DI   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)           //PB12  输入

#define DO_H  GPIO_SetBits(GPIOB, GPIO_Pin_14)       //命令位高
#define DO_L GPIO_ResetBits(GPIOB, GPIO_Pin_14)      //命令位低

#define CS_H GPIO_SetBits(GPIOB, GPIO_Pin_13)      //CS拉高
#define CS_L  GPIO_ResetBits(GPIOB, GPIO_Pin_13)      //CS拉低

#define CLK_H GPIO_SetBits(GPIOB, GPIO_Pin_12)     //时钟拉高
#define CLK_L GPIO_ResetBits(GPIOB, GPIO_Pin_12)     //时钟拉低

//按键定义
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

#define PSS_RX 5	//右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);			//判断是否为红灯模式
void PS2_ReadData(void);
void PS2_Cmd(u8 CMD);
u8 PS2_DataKey(void);			//键值读取
u8 PS2_AnologData(u8 button);	//得到一个摇杆的模拟量
void PS2_ClearData(void);		//清除数据缓冲区
#endif





