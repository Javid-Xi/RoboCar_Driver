/**
************************************************************
* @file         adc.c
* @brief        adc + dma 多通道读取 电量读取
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
* Description: 初始化 
* Parameter: none
* Return: 1:success 0:fail
* Note: DMA_BufferSize:开几路就等于多少
*		DMA_Memory0BaseAddr内存地址
*		ADC_RegularChannelConfig 开几路加几个
*************************************************/
void ADC_DMA_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    /***************I/O口设置************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能GPIOB
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //设置成模拟输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB0

    /***************DMA设置***************************/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//启动DMA时钟
    DMA_DeInit(DMA1_Channel1);                           //DMA通道配置

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);//外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Val; //内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//dma传输方向
    DMA_InitStructure.DMA_BufferSize = 1;//设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//设置DMA的内存递增模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存数据字长
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//设置DMA的传输模式 循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//设置DMA的优先级别
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//使能DMA通道的内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道

    DMA_Cmd(DMA1_Channel1, ENABLE);      //使能通道

    /**********************ADC配置**************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);//启动ADC时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//独立工作模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //连续转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;//用于转换的通道数
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5); //通道一转换结果保存到ADC_Val[0] 采样时间越长越精准

    ADC_DMACmd(ADC1, ENABLE);//开启ADC的DMA支持
    ADC_Cmd(ADC1, ENABLE);

    //校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*************************************************
* Function: ADC_Get_power
* Description: 返回电压 
* Parameter: none
* Return: 返回电压值
*************************************************/
uint16_t ADC_Get_power(void)
{
    return ADC_Val[0];
}


