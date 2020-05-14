/**
************************************************************
* @file         iic.c
* @brief        ģ��iic����
* @author       Javid
* @date         2020-03-20
* @version      1.0
*
***********************************************************/

#include "iic.h"
#include "delay.h"

/*************************************************
* Function: IIC_Init
* Description: ��ʼ��IIC
* Parameter: none
* Return: none
*************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //��ʹ������IO PORTBʱ��

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	 // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);						 //PB10,PB11 �����
}

/*************************************************
* Function: IIC_Start
* Description: ����IIC��ʼ�ź�
* Parameter: none
* Return: none
*************************************************/
void IIC_Start(void)
{
    SDA_OUT();     //sda�����
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(2);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    delay_us(2);
    IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
}

/*************************************************
* Function: IIC_Stop
* Description: ����IICֹͣ�ź�
* Parameter: none
* Return: none
*************************************************/
void IIC_Stop(void)
{
    SDA_OUT();//sda�����
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    delay_us(2);
    IIC_SCL = 1;
    IIC_SDA = 1; //����I2C���߽����ź�
    delay_us(2);
}

/*************************************************
* Function: IIC_Wait_Ack
* Description: �ȴ�Ӧ���źŵ���
* Parameter: none
* Return: 1:����Ӧ��ʧ��; 0:����Ӧ��ɹ�
*************************************************/
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN();      //SDA����Ϊ����
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    while(READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL = 0; //ʱ�����0
    return 0;
}

/*************************************************
* Function: IIC_Ack
* Description: ����ACKӦ��
* Parameter: none
* Return: none
*************************************************/
void IIC_Ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

/*************************************************
* Function: IIC_NAck
* Description: ������ACKӦ��
* Parameter: none
* Return: none
*************************************************/
void IIC_NAck(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

/*************************************************
* Function: IIC_Send_Byte
* Description: IIC����һ���ֽ�,���شӻ�����Ӧ��
* Parameter: @txd: ���͵��ֽ�
* Return: none
*************************************************/
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
    for(t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}

/*************************************************
* Function: IIC_Read_Byte
* Description: ��1���ֽ�
* Parameter: @ack: ack=1ʱ,����ACK;ack=0,����nACK
* Return: @receive: ��ȡ�����ֽ�
*************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA����Ϊ����
    for(i = 0; i < 8; i++ )
    {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1;
        if(READ_SDA)receive++;
        delay_us(2);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}

