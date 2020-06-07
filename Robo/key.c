/**
************************************************************
* @file         key.c
* @brief        按键中断驱动 控制底盘模式
* @author       Javid
* @date         2020-01-20
* @version      1.0
*
***********************************************************/

#include "key.h"
#include "delay.h"

//底盘控制模式
uint8_t mode = 0;

/*************************************************
* Function: KEY_Init
* Description: 按键中断初始化
* Parameter: none
* Return: none
* Note: 按键接地，上拉输入
*************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //使能GPIOB AFIO复用时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成下拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIOB.8 中断线以及中断初始化配置
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//使能PB8外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);

}

/*************************************************
* Function: EXTI9_5_IRQHandler
* Description: 外部中断1服务程序
* Parameter: none
* Return: none
* Note: 中断向量在startup_stm32f10x_hd.s中
*************************************************/
void EXTI9_5_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE1上的中断标志位
    delay_ms(20);//消抖
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1)
    {
        mode = !mode;
    }
}
