/**
************************************************************
* @file         key.c
* @brief        按键中断驱动 控制底盘控制模式
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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); //使能GPIOA AFIO复用时钟

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIOB.1 中断线以及中断初始化配置   下降沿触发
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);	 //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;				//使能按键KEY2所在的外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	//抢占优先级1，
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;			//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

}

/*************************************************
* Function: EXTI1_IRQHandler
* Description: 外部中断1服务程序
* Parameter: none
* Return: none
* Note: 中断向量在startup_stm32f10x_hd.s中
*************************************************/
void EXTI1_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line1);  //清除LINE1上的中断标志位
    delay_ms(20);//消抖
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)	 //按键KEY2
    {
        mode = !mode;
    }
}

/*************************************************
* Function: Mode_get
* Description: 返回mode值
* Parameter: none
* Return: none
*************************************************/
uint8_t Mode_get(void)
{
    return mode;
}
