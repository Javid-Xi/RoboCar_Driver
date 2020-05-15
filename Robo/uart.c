/**
************************************************************
* @file         uart.c
* @brief        ����DMAͨ��
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

//send_data	uart_send_data;//���ݷ���
extern rcv_data	uart_rcv_data;//���ݽ���

/*************************************************
* Function: UART_Init
* Description: uart1 dma���� ��ʼ��
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

    /* GPIO���� */
    GPIO_InitStructure.GPIO_Pin = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
    GPIO_InitStructure.GPIO_Pin = USARTz_TxPin;
    GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);

    /* NVIC���� */
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

    /* DMA���� */
    DMA_Cmd(USARTz_Tx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(USARTz_Tx_DMA_Channe);//�ָ�ȱʡ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//���ڷ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzTxBuffer;//���ͻ����׵�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//��������ΪĿ��
    DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//��Ҫ���͵��ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USARTz_Tx_DMA_Channe, &DMA_InitStructure);//д������
    DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //���DMA���б�־
    //DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);
    DMA_ITConfig(USARTz_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //����DMA����ͨ���ж�

    /*USARTz_Rx_DMA_Channe*/
    DMA_Cmd(USARTz_Rx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(USARTz_Rx_DMA_Channe);//�ָ�ȱʡ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//���ô��ڽ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzRxBuffer;//���ջ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//��������Ϊ����Դ
    DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//��Ҫ���յ��ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(USARTz_Rx_DMA_Channe, &DMA_InitStructure);//д������
    DMA_ClearFlag(USARTz_Rx_DMA_FLAG);    //���DMA���б�־
    DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE); //����DMA����ͨ��

    //  ��������
    USARTz_InitStructure.USART_BaudRate = 115200;
    USARTz_InitStructure.USART_WordLength = USART_WordLength_8b;
    USARTz_InitStructure.USART_StopBits = USART_StopBits_1;
    USARTz_InitStructure.USART_Parity = USART_Parity_No;
    USARTz_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USARTz_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTz, &USARTz_InitStructure);

    USART_ITConfig(USARTz, USART_IT_IDLE, ENABLE); //�������ڿ���IDLE�ж�
    USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USARTz, ENABLE);

    USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
}


/*************************DMA����*****************************/

/*************************************************
* Function: DMA1_Channel4_IRQHandler
* Description: dma1ͨ��4�жϷ�����
* Parameter: none
* Return: none
* Note: �ж�����������startup_stm32f10x.s�ļ���
*************************************************/
void DMA1_Channel4_IRQHandler(void)
{
    //�ж��Ƿ������
    if(DMA_GetITStatus(DMA1_FLAG_TC4))
    {
        DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //���DMA���б�־
        DMA_Cmd(USARTz_Tx_DMA_Channe, DISABLE);  //�ر�DMA����ͨ��
    }
}

/*************************************************
* Function: UART_DMA_Start_tx
* Description: ����DMA����
* Parameter: size,�������ݳ���
* Return: none
*************************************************/
void UART_DMA_Start_tx(uint8_t size)
{
    USARTz_Tx_DMA_Channe->CNDTR = (uint16_t)size; //���¸�ֵ ָ�����ͻ��泤��
    DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);  //����DMA����
}

/*************************DMA����*****************************/

/*************************************************
* Function: USART1_IRQHandler
* Description: �����жϷ�����
* Parameter: none
* Return: none
* Note: �ж�����������startup_stm32f10x.s��
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
* Description: ����DMA����
* Parameter: none
* Return: none
*************************************************/
void UART_DMA_Read(void)
{
    uint8_t rxcounter;
    uint8_t i;
    DMA_Cmd(USARTz_Rx_DMA_Channe, DISABLE);    //�ر�DMA��ֹ����
    DMA_ClearFlag( USARTz_Rx_DMA_FLAG );   //�����־λ
    rxcounter = USARTzTxBufferSize - DMA_GetCurrDataCounter(USARTz_Rx_DMA_Channe); //��ȡ���յ����ֽ���
    USARTz_Rx_DMA_Channe->CNDTR = USARTzTxBufferSize; //���¸�ֵ����ֵ
    memset(USARTzRxBufferD, 0, sizeof(USARTzRxBufferD));

    for(i = 0; i < rxcounter; i++) {
        USARTzRxBufferD[i] = USARTzRxBuffer[i];//��ȡ���յ������ݣ���������RxBufferD��
    }
    //�����յ������ݰ�
    if(UART_data_check(USARTzRxBufferD) == 0)
        printf("data analyze error\n\r");

    for(i = 0; i < rxcounter; i++)
        USARTzRxBuffer[i] = 0;//clear Rx buffer
    DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);  //DMA���� �ȴ���һ֡����
}


/*************************************************
* Function: UART_data_send
* Description: ���ݴ������
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

    UART_DMA_Start_tx(9);	//���ݰ�����
}

/*************************************************
* Function: USART1_data_check
* Description: ��������У��
* Parameter: *pdata
* Return: 1,success; 0,fail
*************************************************/
int8_t UART_data_check(uint8_t	*pdata)
{
    int8_t	crc = 0;
    int8_t  p_crc = 0;
    if((*(pdata + 0) == 0xff) && (*(pdata + 1) == 0xff)) {
        crc = (*(pdata + 2)) ^ (*(pdata + 3)) ^ (*(pdata + 4));//����������ת��������������
        p_crc = (int8_t)(*(pdata + 5));//����������ת��������������
    }
    else return 0;

    if(p_crc != crc ) return 0;//У��ͷ�������

    //���ݰ�������ȷ����ȡ����
    memset(&uart_rcv_data, 0, sizeof(uart_rcv_data));
    uart_rcv_data.vx = *(pdata + 2);
    uart_rcv_data.vy = *(pdata + 3);
    uart_rcv_data.vw = *(pdata + 4);

    return 1;
}
