/**
************************************************************
* @file         uart.c
* @brief        串口DMA通信
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include "uart.h"
#include "string.h"

uint8_t USARTzTxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBufferD[USARTzTxBufferSize];

//send_data	uart_send_data;//数据发送
extern rcv_data	uart_rcv_data;//数据接收

/*************************************************
* Function: UART_Init
* Description: uart1 dma配置 初始化
* Parameter: none
* Return: none
*************************************************/
void UART_DMA_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(USARTz_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(USARTz_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    USART_InitTypeDef USARTz_InitStructure;

    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_Pin = USARTz_TxPin;
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTz_TX_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* DMA配置 */
    DMA_Cmd(USARTz_Tx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(USARTz_Tx_DMA_Channe);//恢复缺省配置
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzTxBuffer;//发送缓冲首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//设置外设为目标
    DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//需要发送的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式
    DMA_Init(USARTz_Tx_DMA_Channe, &DMA_InitStructure);//写入配置
    DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //清除DMA所有标志
    //DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);
    DMA_ITConfig(USARTz_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //开启DMA发送通道中断

    /*USARTz_Rx_DMA_Channe*/
    DMA_Cmd(USARTz_Rx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(USARTz_Rx_DMA_Channe);//恢复缺省配置
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//设置串口接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzRxBuffer;//接收缓存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//设置外设为数据源
    DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//需要接收的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式
    DMA_Init(USARTz_Rx_DMA_Channe, &DMA_InitStructure);//写入配置
    DMA_ClearFlag(USARTz_Rx_DMA_FLAG);    //清除DMA所有标志
    DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE); //开启DMA接收通道

    //  串口配置
    USARTz_InitStructure.USART_BaudRate = 115200;
    USARTz_InitStructure.USART_WordLength = USART_WordLength_8b;
    USARTz_InitStructure.USART_StopBits = USART_StopBits_1;
    USARTz_InitStructure.USART_Parity = USART_Parity_No;
    USARTz_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USARTz_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTz, &USARTz_InitStructure);

    USART_ITConfig(USARTz, USART_IT_IDLE, ENABLE); //开启串口空闲IDLE中断
    USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USARTz, ENABLE);

    USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
}


/*************************DMA发送*****************************/

/*************************************************
* Function: DMA1_Channel4_IRQHandler
* Description: dma1通道4中断服务函数
* Parameter: none
* Return: none
* Note: 中断向量定义在startup_stm32f10x.s文件中
*************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    //判断是否发送完成
    if(DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //清除DMA所有标志
        DMA_Cmd(USARTz_Tx_DMA_Channe, DISABLE);  //关闭DMA发送通道
    }
}

/*************************************************
* Function: UART_DMA_Start_tx
* Description: 串口DMA发送
* Parameter: size,发送数据长度
* Return: none
*************************************************/
void UART_DMA_Start_tx(uint8_t size)
{
    USARTz_Tx_DMA_Channe->CNDTR = (uint16_t)size; //重新赋值 指定发送缓存长度
    DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);  //开启DMA发送
}

/*************************DMA接收*****************************/

/*************************************************
* Function: USART1_IRQHandler
* Description: 串口中断服务函数
* Parameter: none
* Return: none
* Note: 中断向量定义在startup_stm32f10x.s中
*************************************************/
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USARTz, USART_IT_IDLE) != RESET)
    {
        UART_DMA_Read();
        USART_ReceiveData(USARTz);
    }
}

/*************************************************
* Function: UART_DMA_Read
* Description: 串口DMA接收
* Parameter: none
* Return: none
*************************************************/
void UART_DMA_Read(void)
{
    uint8_t rxcounter;
    uint8_t i;
    DMA_Cmd(USARTz_Rx_DMA_Channe, DISABLE);    //关闭DMA防止干扰
    DMA_ClearFlag( USARTz_Rx_DMA_FLAG );   //清除标志位
    rxcounter = USARTzTxBufferSize - DMA_GetCurrDataCounter(USARTz_Rx_DMA_Channe); //获取接收到的字节数
    USARTz_Rx_DMA_Channe->CNDTR = USARTzTxBufferSize; //重新赋值计数值
    memset(USARTzRxBufferD, 0, sizeof(USARTzRxBufferD));

    for(i = 0; i < rxcounter; i++) {
        USARTzRxBufferD[i] = USARTzRxBuffer[i];//获取接收到的数据，存入数组RxBufferD中
    }
    //分析收到的数据包
    if(UART_data_check(USARTzRxBufferD) == 0)
        printf("data analyze error\n\r");

    for(i = 0; i < rxcounter; i++)
        USARTzRxBuffer[i] = 0;//clear Rx buffer
    DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);  //DMA开启 等待下一帧数据
}


/*************************************************
* Function: UART_data_send
* Description: 数据打包发送
* Parameter: *data
* Return: none
*************************************************/
void UART_data_send(send_data *data)
{
    USARTzTxBuffer[0] = 0xaa;
    USARTzTxBuffer[1] = 0xaa;

    USARTzTxBuffer[2] = data->Speed_A;
    USARTzTxBuffer[3] = data->Speed_B;
    USARTzTxBuffer[4] = data->Speed_C;
    USARTzTxBuffer[5] = data->Speed_D;

    USARTzTxBuffer[6] = data->yaw.cv[0];
    USARTzTxBuffer[7] = data->yaw.cv[1];

    USARTzTxBuffer[8] = USARTzTxBuffer[2] ^ USARTzTxBuffer[3] ^ USARTzTxBuffer[4] ^ USARTzTxBuffer[5] ^
                         USARTzTxBuffer[6] ^ USARTzTxBuffer[7];

    UART_DMA_Start_tx(9);	//数据包发送
}

/*************************************************
* Function: USART1_data_check
* Description: 接收数据校验
* Parameter: *pdata
* Return: 1,success; 0,fail
*************************************************/
int8_t UART_data_check(uint8_t	*pdata)
{
    int8_t	crc = 0;
    int8_t  p_crc = 0;
    if((*(pdata + 0) == 0xff) && (*(pdata + 1) == 0xff)) {
        crc = (*(pdata + 2)) ^ (*(pdata + 3)) ^ (*(pdata + 4));//不进行类型转换，负数不正常
        p_crc = (int8_t)(*(pdata + 5));//不进行类型转换，负数不正常
    }
    else return 0;

    if(p_crc != crc ) return 0;//校验和分析有误

    //数据包分析正确，提取数据
    memset(&uart_rcv_data, 0, sizeof(uart_rcv_data));
    uart_rcv_data.vx = *(pdata + 2);
    uart_rcv_data.vy = *(pdata + 3);
    uart_rcv_data.vw = *(pdata + 4);

    return 1;
}
