/**
************************************************************
* @file         uart_mpu.c
* @brief        串口2DMA通信 读取jy-901模块姿态数据
* @author       Javid
* @date         2020-05-14
* @version      1.0
* @Note			俯仰角（y 轴）Pitch=((PitchH<<8)|PitchL)/32768*180(°)
*				偏航角（z 轴）Yaw=((YawH<<8)|YawL)/32768*180(°)
*				温度计算公式：
*				T=((TH<<8)|TL) /100 ℃
*				校验和：
*				Sum=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+TH+TL
*
***********************************************************/

#include "uart_mpu.h"
#include "string.h"

//数据帧头
u8 acc[2] = {0x55, 0x51};	//加速度
u8 groy[2] = {0x55, 0x52};	//角速度
u8 angle[2] = {0x55, 0x53}; //欧拉角
u8 Quaternion[2] = {0x55, 0x59}; //四元数

uint8_t UART_MPUTxBuffer[UART_MPUTxBufferSize];
uint8_t UART_MPURxBuffer[UART_MPUTxBufferSize];
uint8_t UART_MPURxBufferD[UART_MPUTxBufferSize];

MPU_rcv_data uart_mpu_rcv_data;

/*************************************************
* Function: UART_MPU_DMA_Init
* Description: uart2 dma配置 初始化
* Parameter: none
* Return: none
*************************************************/
void UART_MPU_DMA_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(UART_MPU_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(UART_MPU_CLK, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    USART_InitTypeDef UART_MPU_InitStructure;

    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = UART_MPU_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(UART_MPU_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_InitStructure.GPIO_Pin = UART_MPU_TxPin;
    GPIO_Init(UART_MPU_GPIO, &GPIO_InitStructure);

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTz_TX_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable the UART_MPU Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART_MPU_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* DMA配置 */
    DMA_Cmd(UART_MPU_Tx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(UART_MPU_Tx_DMA_Channe);//恢复缺省配置
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)UART_MPU_DR_Base;//串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART_MPUTxBuffer;//发送缓冲首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//设置外设为目标
    DMA_InitStructure.DMA_BufferSize = UART_MPUTxBufferSize;//需要发送的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式
    DMA_Init(UART_MPU_Tx_DMA_Channe, &DMA_InitStructure);//写入配置
    DMA_ClearFlag(UART_MPU_Tx_DMA_FLAG);    //清除DMA所有标志
    //DMA_Cmd(UART_MPU_Tx_DMA_Channe, ENABLE);
    DMA_ITConfig(UART_MPU_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //开启DMA发送通道中断

    /*UART_MPU_Rx_DMA_Channe*/
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(UART_MPU_Rx_DMA_Channe);//恢复缺省配置
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)UART_MPU_DR_Base;//设置串口接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART_MPURxBuffer;//接收缓存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//设置外设为数据源
    DMA_InitStructure.DMA_BufferSize = UART_MPUTxBufferSize;//需要接收的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式
    DMA_Init(UART_MPU_Rx_DMA_Channe, &DMA_InitStructure);//写入配置
    DMA_ClearFlag(UART_MPU_Rx_DMA_FLAG);    //清除DMA所有标志
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, ENABLE); //开启DMA接收通道

    //  串口配置
    UART_MPU_InitStructure.USART_BaudRate = 115200;
    UART_MPU_InitStructure.USART_WordLength = USART_WordLength_8b;
    UART_MPU_InitStructure.USART_StopBits = USART_StopBits_1;
    UART_MPU_InitStructure.USART_Parity = USART_Parity_No;
    UART_MPU_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    UART_MPU_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART_MPU, &UART_MPU_InitStructure);

    USART_ITConfig(UART_MPU, USART_IT_IDLE, ENABLE); //开启串口空闲IDLE中断
    USART_DMACmd(UART_MPU, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(UART_MPU, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART_MPU, ENABLE);

    USART_DMACmd(UART_MPU, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(UART_MPU, USART_DMAReq_Rx, ENABLE);
}


/*************************DMA发送*****************************/

/*************************************************
* Function: DMA1_Channel7_IRQHandler
* Description: dma1通道7中断服务函数
* Parameter: none
* Return: none
* Note: 中断向量定义在startup_stm32f10x.s文件中
*************************************************/
void DMA1_Channel7_IRQHandler(void)
{
    //判断是否发送完成
    if(DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        DMA_ClearFlag(UART_MPU_Tx_DMA_FLAG);    //清除DMA所有标志
        DMA_Cmd(UART_MPU_Tx_DMA_Channe, DISABLE);  //关闭DMA发送通道
    }
}

/*************************************************
* Function: UART_MPU_DMA_Start
* Description: 串口DMA发送
* Parameter: size,发送数据长度
* Return: none
*************************************************/
void UART_MPU_DMA_Start(uint8_t size)
{
    UART_MPU_Tx_DMA_Channe->CNDTR = (uint16_t)size; //重新赋值 指定发送缓存长度
    DMA_Cmd(UART_MPU_Tx_DMA_Channe, ENABLE);  //开启DMA发送
}

/*************************DMA接收*****************************/

/*************************************************
* Function: USART2_IRQHandler
* Description: 串口2中断服务函数
* Parameter: none
* Return: none
* Note: 中断向量定义在startup_stm32f10x.s中
*************************************************/
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(UART_MPU, USART_IT_IDLE) != RESET)
    {
        UART_MPU_DMA_Read();
        USART_ReceiveData(UART_MPU);
    }
}

/*************************************************
* Function: UART_MPU_DMA_Read
* Description: 串口2DMA接收
* Parameter: none
* Return: none
*************************************************/
void UART_MPU_DMA_Read(void)
{
    uint8_t rxcounter;
    uint8_t i;
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, DISABLE);    //关闭DMA防止干扰
    DMA_ClearFlag( UART_MPU_Rx_DMA_FLAG );   //清除标志位
    rxcounter = UART_MPUTxBufferSize - DMA_GetCurrDataCounter(UART_MPU_Rx_DMA_Channe); //获取接收到的字节数
    UART_MPU_Rx_DMA_Channe->CNDTR = UART_MPUTxBufferSize; //重新赋值计数值
    memset(UART_MPURxBufferD, 0, sizeof(UART_MPURxBufferD));

    for(i = 0; i < rxcounter; i++) {
        UART_MPURxBufferD[i] = UART_MPURxBuffer[i];//获取接收到的数据，存入数组RxBufferD中
    }
    //分析收到的数据包
    if(UART_MPU_data_check(UART_MPURxBufferD) == 0)
        printf("data analyze error\n\r");

    for(i = 0; i < rxcounter; i++)
        UART_MPURxBuffer[i] = 0;//clear Rx buffer
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, ENABLE);  //DMA开启 等待下一帧数据
}


/*************************************************
* Function: UART_MPU_data_send
* Description: 数据打包发送
* Parameter: mode: 0,请求欧拉角;1,请求四元数
* Return: none
*************************************************/
void UART_MPU_data_send(u8 mode)
{
    UART_MPUTxBuffer[0] = 0xff;
    UART_MPUTxBuffer[1] = 0xaa;
    UART_MPUTxBuffer[2] = 0x02;

    if(mode == 0)
    {
        //只输出欧拉角
        UART_MPUTxBuffer[3] = 0x08;
        UART_MPUTxBuffer[4] = 0x00;
    }
    else
    {
        //只输出四元数
        UART_MPUTxBuffer[3] = 0x00;
        UART_MPUTxBuffer[4] = 0x02;
    }

    UART_MPU_DMA_Start(5);	//数据包发送
}

/*************************************************
* Function: UART_MPU_data_check
* Description: 接收数据校验
* Parameter: *pdata
* Return: 1,success; 0,fail
*************************************************/
int8_t UART_MPU_data_check(uint8_t	*pdata)
{
    int8_t	crc = 0;
    int8_t  p_crc = 0;
    if(*(pdata + 0) == 0x55 )
    {
        crc = (*(pdata + 2)) + (*(pdata + 3)) + (*(pdata + 4)) + (*(pdata + 5)) +
              (*(pdata + 6)) + (*(pdata + 7)) + (*(pdata + 8)) + (*(pdata + 9));//校验和

        p_crc = (int8_t)(*(pdata + 10));//不进行类型转换，负数不正常
    }
    else return 0;

    if(p_crc != crc ) return 0;//校验和分析有误

    //数据包分析正确，提取数据
    memset(&uart_mpu_rcv_data, 0, sizeof(uart_mpu_rcv_data));

    if(*(pdata + 1) == angle[1]) //数据帧为欧拉角
    {
        uart_mpu_rcv_data.pitch.cv[0] = *(pdata + 2);
        uart_mpu_rcv_data.pitch.cv[1] = *(pdata + 3);

        uart_mpu_rcv_data.roll.cv[0] = *(pdata + 4);
        uart_mpu_rcv_data.roll.cv[1] = *(pdata + 5);

        uart_mpu_rcv_data.yaw.cv[0] = *(pdata + 6);
        uart_mpu_rcv_data.yaw.cv[1] = *(pdata + 7);

        uart_mpu_rcv_data.temp.cv[0] = *(pdata + 8);
        uart_mpu_rcv_data.temp.cv[1] = *(pdata + 9);
    }

    else if(*(pdata + 1) == Quaternion[1]) //数据帧为四元数
    {
        uart_mpu_rcv_data.q0.cv[0] = *(pdata + 2);
        uart_mpu_rcv_data.q0.cv[1] = *(pdata + 3);

        uart_mpu_rcv_data.q1.cv[0] = *(pdata + 4);
        uart_mpu_rcv_data.q1.cv[1] = *(pdata + 5);

        uart_mpu_rcv_data.q2.cv[0] = *(pdata + 6);
        uart_mpu_rcv_data.q2.cv[1] = *(pdata + 7);

        uart_mpu_rcv_data.q3.cv[0] = *(pdata + 8);
        uart_mpu_rcv_data.q3.cv[1] = *(pdata + 9);
    }
    return 1;
}
