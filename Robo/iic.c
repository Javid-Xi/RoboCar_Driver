/**
************************************************************
* @file         iic.c
* @brief        模拟iic驱动
* @author       Javid
* @date         2020-03-20
* @version      1.0
*
***********************************************************/

#include "iic.h"
#include "delay.h"

/*************************************************
* Function: IIC_Init
* Description: 初始化IIC
* Parameter: none
* Return: none
*************************************************/
void IIC_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //先使能外设IO PORTB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	 // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);						 //PB10,PB11 输出高
}

/*************************************************
* Function: IIC_Start
* Description: 产生IIC起始信号
* Parameter: none
* Return: none
*************************************************/
void IIC_Start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(2);
    IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
    delay_us(2);
    IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}

/*************************************************
* Function: IIC_Stop
* Description: 产生IIC停止信号
* Parameter: none
* Return: none
*************************************************/
void IIC_Stop(void)
{
    SDA_OUT();//sda线输出
    IIC_SCL = 0;
    IIC_SDA = 0; //STOP:when CLK is high DATA change form low to high
    delay_us(2);
    IIC_SCL = 1;
    IIC_SDA = 1; //发送I2C总线结束信号
    delay_us(2);
}

/*************************************************
* Function: IIC_Wait_Ack
* Description: 等待应答信号到来
* Parameter: none
* Return: 1:接收应答失败; 0:接收应答成功
*************************************************/
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN();      //SDA设置为输入
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
    IIC_SCL = 0; //时钟输出0
    return 0;
}

/*************************************************
* Function: IIC_Ack
* Description: 产生ACK应答
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
* Description: 不产生ACK应答
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
* Description: IIC发送一个字节,返回从机有无应答
* Parameter: @txd: 发送的字节
* Return: none
*************************************************/
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL = 0; //拉低时钟开始数据传输
    for(t = 0; t < 8; t++)
    {
        IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}

/*************************************************
* Function: IIC_Read_Byte
* Description: 读1个字节
* Parameter: @ack: ack=1时,发送ACK;ack=0,发送nACK
* Return: @receive: 读取到的字节
*************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}

