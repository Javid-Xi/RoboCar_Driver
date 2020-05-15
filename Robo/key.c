/**
************************************************************
* @file         key.c
* @brief        �����ж����� ���Ƶ��̿���ģʽ
* @author       Javid
* @date         2020-01-20
* @version      1.0
*
***********************************************************/

#include "key.h"
#include "delay.h"

//���̿���ģʽ
uint8_t mode = 0;

/*************************************************
* Function: KEY_Init
* Description: �����жϳ�ʼ��
* Parameter: none
* Return: none
* Note: �����ӵأ���������
*************************************************/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //ʹ��GPIOA AFIO����ʱ��

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIOB.1 �ж����Լ��жϳ�ʼ������   �½��ش���
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);	 //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;				//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

}

/*************************************************
* Function: EXTI1_IRQHandler
* Description: �ⲿ�ж�1�������
* Parameter: none
* Return: none
* Note: �ж�������startup_stm32f10x_hd.s��
*************************************************/
void EXTI1_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line1);  //���LINE1�ϵ��жϱ�־λ
    delay_ms(20);//����
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)	 //����KEY2
    {
        mode = !mode;
    }
}

/*************************************************
* Function: Mode_get
* Description: ����modeֵ
* Parameter: none
* Return: none
*************************************************/
uint8_t Mode_get(void)
{
    return mode;
}
