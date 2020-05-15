/**
************************************************************
* @file         uart_mpu.c
* @brief        ����2DMAͨ�� ��ȡjy-901ģ����̬����
* @author       Javid
* @date         2020-05-14
* @version      1.0
* @Note			�����ǣ�y �ᣩPitch=((PitchH<<8)|PitchL)/32768*180(��)
*				ƫ���ǣ�z �ᣩYaw=((YawH<<8)|YawL)/32768*180(��)
*				�¶ȼ��㹫ʽ��
*				T=((TH<<8)|TL) /100 ��
*				У��ͣ�
*				Sum=0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+TH+TL
*
***********************************************************/

#include "uart_mpu.h"
#include "string.h"

//����֡ͷ
u8 acc[2] = {0x55, 0x51};	//���ٶ�
u8 groy[2] = {0x55, 0x52};	//���ٶ�
u8 angle[2] = {0x55, 0x53}; //ŷ����
u8 Quaternion[2] = {0x55, 0x59}; //��Ԫ��

uint8_t UART_MPUTxBuffer[UART_MPUTxBufferSize];
uint8_t UART_MPURxBuffer[UART_MPUTxBufferSize];
uint8_t UART_MPURxBufferD[UART_MPUTxBufferSize];

MPU_rcv_data uart_mpu_rcv_data;

/*************************************************
* Function: UART_MPU_DMA_Init
* Description: uart2 dma���� ��ʼ��
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

    /* GPIO���� */
    GPIO_InitStructure.GPIO_Pin = UART_MPU_RxPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(UART_MPU_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
    GPIO_InitStructure.GPIO_Pin = UART_MPU_TxPin;
    GPIO_Init(UART_MPU_GPIO, &GPIO_InitStructure);

    /* NVIC���� */
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

    /* DMA���� */
    DMA_Cmd(UART_MPU_Tx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(UART_MPU_Tx_DMA_Channe);//�ָ�ȱʡ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)UART_MPU_DR_Base;//���ڷ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART_MPUTxBuffer;//���ͻ����׵�ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//��������ΪĿ��
    DMA_InitStructure.DMA_BufferSize = UART_MPUTxBufferSize;//��Ҫ���͵��ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(UART_MPU_Tx_DMA_Channe, &DMA_InitStructure);//д������
    DMA_ClearFlag(UART_MPU_Tx_DMA_FLAG);    //���DMA���б�־
    //DMA_Cmd(UART_MPU_Tx_DMA_Channe, ENABLE);
    DMA_ITConfig(UART_MPU_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //����DMA����ͨ���ж�

    /*UART_MPU_Rx_DMA_Channe*/
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, DISABLE); //stop dma
    DMA_DeInit(UART_MPU_Rx_DMA_Channe);//�ָ�ȱʡ����
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)UART_MPU_DR_Base;//���ô��ڽ������ݼĴ���
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART_MPURxBuffer;//���ջ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//��������Ϊ����Դ
    DMA_InitStructure.DMA_BufferSize = UART_MPUTxBufferSize;//��Ҫ���յ��ֽ���
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�������ӵ���
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ滺������ַ���ӵ���
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //�������ݿ�� һ���ֽ�
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //�ڴ����ݿ��һ���ֽ�
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //���δ���ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //�����ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //�ر��ڴ浽�ڴ��DMAģʽ
    DMA_Init(UART_MPU_Rx_DMA_Channe, &DMA_InitStructure);//д������
    DMA_ClearFlag(UART_MPU_Rx_DMA_FLAG);    //���DMA���б�־
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, ENABLE); //����DMA����ͨ��

    //  ��������
    UART_MPU_InitStructure.USART_BaudRate = 115200;
    UART_MPU_InitStructure.USART_WordLength = USART_WordLength_8b;
    UART_MPU_InitStructure.USART_StopBits = USART_StopBits_1;
    UART_MPU_InitStructure.USART_Parity = USART_Parity_No;
    UART_MPU_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    UART_MPU_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART_MPU, &UART_MPU_InitStructure);

    USART_ITConfig(UART_MPU, USART_IT_IDLE, ENABLE); //�������ڿ���IDLE�ж�
    USART_DMACmd(UART_MPU, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(UART_MPU, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(UART_MPU, ENABLE);

    USART_DMACmd(UART_MPU, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(UART_MPU, USART_DMAReq_Rx, ENABLE);
}


/*************************DMA����*****************************/

/*************************************************
* Function: DMA1_Channel7_IRQHandler
* Description: dma1ͨ��7�жϷ�����
* Parameter: none
* Return: none
* Note: �ж�����������startup_stm32f10x.s�ļ���
*************************************************/
void DMA1_Channel7_IRQHandler(void)
{
    //�ж��Ƿ������
    if(DMA_GetITStatus(DMA1_FLAG_TC7))
    {
        DMA_ClearFlag(UART_MPU_Tx_DMA_FLAG);    //���DMA���б�־
        DMA_Cmd(UART_MPU_Tx_DMA_Channe, DISABLE);  //�ر�DMA����ͨ��
    }
}

/*************************************************
* Function: UART_MPU_DMA_Start
* Description: ����DMA����
* Parameter: size,�������ݳ���
* Return: none
*************************************************/
void UART_MPU_DMA_Start(uint8_t size)
{
    UART_MPU_Tx_DMA_Channe->CNDTR = (uint16_t)size; //���¸�ֵ ָ�����ͻ��泤��
    DMA_Cmd(UART_MPU_Tx_DMA_Channe, ENABLE);  //����DMA����
}

/*************************DMA����*****************************/

/*************************************************
* Function: USART2_IRQHandler
* Description: ����2�жϷ�����
* Parameter: none
* Return: none
* Note: �ж�����������startup_stm32f10x.s��
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
* Description: ����2DMA����
* Parameter: none
* Return: none
*************************************************/
void UART_MPU_DMA_Read(void)
{
    uint8_t rxcounter;
    uint8_t i;
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, DISABLE);    //�ر�DMA��ֹ����
    DMA_ClearFlag( UART_MPU_Rx_DMA_FLAG );   //�����־λ
    rxcounter = UART_MPUTxBufferSize - DMA_GetCurrDataCounter(UART_MPU_Rx_DMA_Channe); //��ȡ���յ����ֽ���
    UART_MPU_Rx_DMA_Channe->CNDTR = UART_MPUTxBufferSize; //���¸�ֵ����ֵ
    memset(UART_MPURxBufferD, 0, sizeof(UART_MPURxBufferD));

    for(i = 0; i < rxcounter; i++) {
        UART_MPURxBufferD[i] = UART_MPURxBuffer[i];//��ȡ���յ������ݣ���������RxBufferD��
    }
    //�����յ������ݰ�
    if(UART_MPU_data_check(UART_MPURxBufferD) == 0)
        printf("data analyze error\n\r");

    for(i = 0; i < rxcounter; i++)
        UART_MPURxBuffer[i] = 0;//clear Rx buffer
    DMA_Cmd(UART_MPU_Rx_DMA_Channe, ENABLE);  //DMA���� �ȴ���һ֡����
}


/*************************************************
* Function: UART_MPU_data_send
* Description: ���ݴ������
* Parameter: mode: 0,����ŷ����;1,������Ԫ��
* Return: none
*************************************************/
void UART_MPU_data_send(u8 mode)
{
    UART_MPUTxBuffer[0] = 0xff;
    UART_MPUTxBuffer[1] = 0xaa;
    UART_MPUTxBuffer[2] = 0x02;

    if(mode == 0)
    {
        //ֻ���ŷ����
        UART_MPUTxBuffer[3] = 0x08;
        UART_MPUTxBuffer[4] = 0x00;
    }
    else
    {
        //ֻ�����Ԫ��
        UART_MPUTxBuffer[3] = 0x00;
        UART_MPUTxBuffer[4] = 0x02;
    }

    UART_MPU_DMA_Start(5);	//���ݰ�����
}

/*************************************************
* Function: UART_MPU_data_check
* Description: ��������У��
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
              (*(pdata + 6)) + (*(pdata + 7)) + (*(pdata + 8)) + (*(pdata + 9));//У���

        p_crc = (int8_t)(*(pdata + 10));//����������ת��������������
    }
    else return 0;

    if(p_crc != crc ) return 0;//У��ͷ�������

    //���ݰ�������ȷ����ȡ����
    memset(&uart_mpu_rcv_data, 0, sizeof(uart_mpu_rcv_data));

    if(*(pdata + 1) == angle[1]) //����֡Ϊŷ����
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

    else if(*(pdata + 1) == Quaternion[1]) //����֡Ϊ��Ԫ��
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
