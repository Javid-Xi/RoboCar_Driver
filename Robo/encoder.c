/**
************************************************************
* @file         encoder.c
* @brief        ��ʱ��������ģʽ����
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include "encoder.h" 

/*************************************************
* Function: ENCODER_AB_Init
* Description: ������AB��ʼ��
* Parameter: cycle����������
* Return: none
*************************************************/
void ENCODER_AB_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	//Reset counter
	TIM2->CNT = 0;
	TIM_Cmd(TIM2, ENABLE);   
}

/*************************************************
* Function: ENCODER_AB_GetCounter
* Description: ������AB��ȡ��������ֵ
* Parameter: none
* Return: ��������ǰֵ
*************************************************/
uint16_t ENCODER_AB_GetCounter(void)
{
	return (TIM_GetCounter(TIM2)); 
}

/*************************************************
* Function: ENCODER_AB_SetCounter
* Description: ������AB���ü�������ֵ
* Parameter: count:��������ֵ
* Return: none
*************************************************/
void ENCODER_AB_SetCounter(uint16_t count)
{
	TIM2->CNT = count;
}

/*************************************************
* Function: ENCODER_CD_Init
* Description: ������CD��ʼ��
* Parameter: cycle:��������
* Return: none
*************************************************/
void ENCODER_CD_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3 , ENABLE); //���������ӳ�书�ܺ���

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//Reset counter
	TIM3->CNT = 0;
	TIM_Cmd(TIM3, ENABLE);  
}

/*************************************************
* Function: ENCODER_CD_GetCounter
* Description: ������CD��ȡ��������ֵ
* Parameter: none
* Return: ��������ǰֵ
*************************************************/
uint16_t ENCODER_CD_GetCounter(void)
{
	return (TIM_GetCounter(TIM3)); 
}

/*************************************************
* Function: ENCODER_CD_SetCounter
* Description: ������CD���ü�������ֵ
* Parameter: count:��������ֵ
* Return: none
*************************************************/
void ENCODER_CD_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}

/*************************************************
* Function: ENCODER_EF_Init
* Description: ������EF��ʼ��
* Parameter: cycle:��������
* Return: none
*************************************************/
void ENCODER_EF_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	//Reset counter
	TIM4->CNT = 0;
	TIM_Cmd(TIM4, ENABLE);     
}

/*************************************************
* Function: ENCODER_EF_GetCounter
* Description: ������EF��ȡ��������ֵ
* Parameter: none
* Return: ��������ǰֵ
*************************************************/
uint16_t ENCODER_EF_GetCounter(void)
{
	return (TIM_GetCounter(TIM4)); 
}

/*************************************************
* Function: ENCODER_EF_SetCounter
* Description: ������EF���ü�������ֵ
* Parameter: count:��������ֵ
* Return: none
*************************************************/
void ENCODER_EF_SetCounter(uint16_t count)
{
	TIM4->CNT = count;
}

/*************************************************
* Function: ENCODER_GH_Init
* Description: ������GH��ʼ��
* Parameter: cycle:��������
* Return: none
*************************************************/
void ENCODER_GH_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	//Reset counter
	TIM5->CNT = 0;
	TIM_Cmd(TIM5, ENABLE);    
}

/*************************************************
* Function: ENCODER_GH_GetCounter
* Description: ������GH��ȡ��������ֵ
* Parameter: none
* Return: ��������ǰֵ
*************************************************/
uint16_t ENCODER_GH_GetCounter(void)
{
	return (TIM_GetCounter(TIM5)); 
}

/*************************************************
* Function: ENCODER_GH_SetCounter
* Description: ������GH���ü�������ֵ
* Parameter: count:��������ֵ
* Return: none
*************************************************/
void ENCODER_GH_SetCounter(uint16_t count)
{
	TIM5->CNT = count;
}
