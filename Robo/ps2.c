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

#include "ps2.h"
#include "delay.h"

u16 Handkey;
u8 Comd[2]={0x01,0x42};	//开始命令。请求数据
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明

/*************************************************
* Function: PS2_Init
* Description: PS2手柄初始化
* Parameter: none
* Return: none
*************************************************/
void PS2_Init(void)
{ 	 											  
	GPIO_InitTypeDef GPIO_InitStructure;         
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/*************************************************
* Function: PS2_Cmd
* Description: 向手柄发送命令
* Parameter: cmd
* Return: none
*************************************************/
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //输出以为控制位
		}
		else DO_L;

		CLK_H;                        //时钟拉高
		delay_us(50);
		CLK_L;
		delay_us(50);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
}

/*************************************************
* Function: PS2_RedLight
* Description: 判断是否为红灯模式
* Parameter: none
* Return: 0,红灯模式; 1,其他模式;
*************************************************/
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}

/*************************************************
* Function: PS2_ReadData
* Description: 读取手柄数据
* Parameter: none
* Return: none
*************************************************/
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //开始命令
	PS2_Cmd(Comd[1]);  //请求数据

	for(byte=2;byte<9;byte++)          //开始接受数据
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			CLK_L;
			delay_us(50);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(50);
	}
	CS_H;	
}

/*************************************************
* Function: PS2_DataKey
* Description: 对读出来的PS2的数据进行处理  只处理了按键部分
* Parameter: none
* Return: 按下的按键值
* Note: 默认数据是红灯模式  只有一个按键按下时,
*		0,没有按下;1,按下
*************************************************/
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0,未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //没有任何按键按下
}

/*************************************************
* Function: PS2_AnologData
* Description: 得到一个摇杆的模拟量	范围0~256
* Parameter: button
* Return: 对应数组数据
*************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

/*************************************************
* Function: PS2_ClearData
* Description: 清除数据缓冲区
* Parameter: none
* Return: none
*************************************************/
void PS2_ClearData(void)
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}






