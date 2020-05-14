/**
************************************************************
* @file         led.c
* @brief        led«˝∂Ø
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#include "led.h"

/*************************************************
* Function: LED_Init
* Description: LED/BEEP ≥ı ºªØ	
* Parameter: none
* Return: none
*************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //LED GPIO≈‰÷√
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //πÿ±’LEDµ∆
    GPIO_SetBits(GPIOA, GPIO_Pin_8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //πÿ±’∑‰√˘∆˜
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
}
