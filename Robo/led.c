/**
************************************************************
* @file         led.c
* @brief        led����
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#include "led.h"
#include "delay.h"

/*************************************************
* Function: LED_Init
* Description: LED/BEEP ��ʼ��
* Parameter: none
* Return: none
*************************************************/
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //LED GPIO����
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PIN_Port, &GPIO_InitStructure);

    //�ر�LED��
    GPIO_SetBits(LED_PIN_Port, LED_PIN);

	//BEEP_GPIO����
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_Init(BEEP_PIN_Port, &GPIO_InitStructure);

    //�رշ�����
    GPIO_SetBits(BEEP_PIN_Port, BEEP_PIN);
}

/*************************************************
* Function: Sysinit_Complete
* Description: ϵͳ��ʼ���ź�
* Parameter: none
* Return: none
*************************************************/
void Sysinit_Complete(void)
{
    uint8_t i;
    for(i = 0; i < 3; i++)
    {
        GPIO_ResetBits(BEEP_PIN_Port, BEEP_PIN);
        GPIO_ResetBits(LED_PIN_Port, LED_PIN);
        delay_ms(100);
		GPIO_SetBits(BEEP_PIN_Port, BEEP_PIN);
        GPIO_SetBits(LED_PIN_Port, LED_PIN);
		delay_ms(100);
    }
}


