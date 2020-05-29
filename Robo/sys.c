/**
************************************************************
* @file         sys.c
* @brief        仿真调试配置
* @author       Javid
* @date         2020-05-08
* @version      1.0
* @Note			JTAG SWD设置
*				位操作设置
***********************************************************/

#include "sys.h"

/*************************************************
* Function: JTAG_Set
* Description: JTAG模式设置	
* Parameter:	mode:jtag,swd模式设置;
				00,全使能;
				01,使能SWD;
				10,全关闭;
				JTAG_SWD_DISABLE   0X02
				SWD_ENABLE         0X01
				JTAG_SWD_ENABLE    0X00	
* Return: none
*************************************************/
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 
