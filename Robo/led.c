/**
************************************************************
* @file         led.c
* @brief        led驱动
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#include "led.h"
#include "delay.h"

/*************************************************
* Function: LED_Init
* Description: LED/BEEP 初始化
* Parameter: none
* Return: none
*************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //LED GPIO配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //关闭LED灯
    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //关闭蜂鸣器
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
}

/*************************************************
* Function: Sysinit_Complete
* Description: 系统初始化信号
* Parameter: none
* Return: none
*************************************************/
void Sysinit_Complete(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    delay_ms(100);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    delay_ms(100);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    delay_ms(100);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    delay_ms(100);
    GPIO_ResetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);
    delay_ms(100);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
}


