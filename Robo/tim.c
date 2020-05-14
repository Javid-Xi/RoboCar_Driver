/**
************************************************************
* @file         tim.c
* @brief        ��ʱ������
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#include "tim.h" 

//�ж�ѭ��״̬���Ʊ�־λ
uint8_t flag_tim = 0;

/*************************************************
* Function: TIM6_Init
* Description: TIM6 ��ʱ����ʼ��������жϣ�
* Parameter: cnt_us �����������ֵ����λ1us����Χ0-65535 
* Return: none
*************************************************/
void TIM6_Init(uint16_t cnt_us)
{ 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); //ʹ��ʱ��
	
	 // �ۼ� TIM_Period�������һ�����»����ж�
	TIM_TimeBaseInitStructure.TIM_Period = cnt_us-1;  //�Զ���װ��ֵ�����65535
	// �趨��ʱ��Ƶ��Ϊ=TIMxCLK/(TIM_Prescaler+1)=1000000Hz
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1; //��ʱ����Ƶ,�������ڣ�period x 1us��
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure); //��ʼ����ʱ��
	
	//�����ʱ�������жϱ�־λ
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	
	//������ʱ�������ж�
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ�� 
	
	// �����ж���Դ
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 	// ������ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		// ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*************************************************
* Function: TIM6_Cmd
* Description: TIM6 ��ʱ�������ر�
* Parameter: NewState: ENABLE or DISABLE
* Return: none
*************************************************/
void TIM6_Cmd(FunctionalState NewState)
{
	TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM6, NewState);
}

/*************************************************
* Function: TIM6_IRQHandler
* Description: TIM6 �жϷ�����
* Parameter: NewState: ENABLE or DISABLE
* Return: none
*************************************************/
void TIM6_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM6, TIM_IT_Update) != RESET ) 
	{	
		//�жϴ�������
		flag_tim = 1;  //��λ10ms��־λ
		
		TIM_ClearITPendingBit(TIM6 , TIM_IT_Update);  		 
	}		 	
}

/*************************************************
* Function: TIM_CheckIrqStatus
* Description: ����Ƿ�����ж�
* Parameter: none
* Return: 0,δ�����ж�; 1,�����ж�
*************************************************/
uint8_t TIM_CheckIrqStatus(void)
{
	//ȷ���ж�,�����������
	if(flag_tim != 0) 
	{
		flag_tim = 0;
		return 1;
	}
	else return 0;

}
