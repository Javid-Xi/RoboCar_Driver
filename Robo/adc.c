/**
************************************************************
* @file         adc.c
* @brief        adc + dma ��ͨ����ȡ ������ȡ
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include "adc.h"
#include "stm32f10x_adc.h"

uint16_t ADC_Val[1];

/*************************************************
* Function: ADC_DMA_Init
* Description: ��ʼ�� 
* Parameter: none
* Return: 1:success 0:fail
* Note: DMA_BufferSize:����·�͵��ڶ���
*		DMA_Memory0BaseAddr�ڴ��ַ
*		ADC_RegularChannelConfig ����·�Ӽ���
*************************************************/
void ADC_DMA_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    /***************I/O������************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��GPIOB
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //���ó�ģ������
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB0

    /***************DMA����***************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//����DMAʱ��
    DMA_DeInit(DMA1_Channel1);                           //DMAͨ������

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);//�����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Val; //�ڴ��ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//dma���䷽��
    DMA_InitStructure.DMA_BufferSize = 1;//����DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//����DMA���������ģʽ��һ������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//����DMA���ڴ����ģʽ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���������ֳ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�ڴ������ֳ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//����DMA�Ĵ���ģʽ ѭ��ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//����DMA�����ȼ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//ʹ��DMAͨ�����ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��

    DMA_Cmd(DMA1_Channel1, ENABLE);      //ʹ��ͨ��

    /**********************ADC����**************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//����ADCʱ��
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//��������ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //����ת��
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 1;//����ת����ͨ����
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5); //ͨ��һת��������浽ADC_Val[0] ����ʱ��Խ��Խ��׼

    ADC_DMACmd(ADC1, ENABLE);//����ADC��DMA֧��
    ADC_Cmd(ADC1, ENABLE);

    //У׼
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*************************************************
* Function: ADC_Get_power
* Description: ���ص�ѹ 
* Parameter: none
* Return: ���ص�ѹֵ
*************************************************/
uint16_t ADC_Get_power(void)
{
    return ADC_Val[0];
}


