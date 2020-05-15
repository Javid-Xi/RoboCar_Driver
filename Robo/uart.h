/**
************************************************************
* @file         uart.c
* @brief        串口DMA通信
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __UART_H
#define __UART_H

#include "sys.h"

#define USARTzTxBufferSize   64
#define USARTz							USART1
#define USARTz_GPIO						GPIOA
#define USARTz_CLK						RCC_APB2Periph_USART1
#define USARTz_GPIO_CLK					RCC_APB2Periph_GPIOA
#define USARTz_RxPin					GPIO_Pin_10
#define USARTz_TxPin					GPIO_Pin_9
#define USARTz_IRQn						USART1_IRQn
#define USARTz_DR_Base					(uint32_t)(&USART1->DR)

#define USARTz_Tx_DMA_Channe			DMA1_Channel4
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL4 //DMA1_FLAG_TC2|DMA1_FLAG_TE2
#define UASRTz_TX_DMA_IRQ				DMA1_Channel4_IRQn

#define USARTz_Rx_DMA_Channe			DMA1_Channel5
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL5 //DMA1_FLAG_TC3 |DMA1_FLAG_TE3
#define USARTz_RX_DMA_IRQ				DMA1_Channel5_IRQn

//浮点数与HEX快速获取
typedef	union {
    float fv;
    uint8_t cv[4];
} float_union;

//short与HEX快速获取
typedef	union {
    short sv;
    uint8_t cv[2];
} short_union;


//接收数据结构
typedef	struct {

    int8_t vx;//线速度x
    int8_t vy;//线速度y
    int8_t vw;//角速度


} rcv_data;

//发送数据结构
typedef	struct {

    //电机编码器读数
    int8_t Speed_A;
    int8_t Speed_B;
    int8_t Speed_C;
    int8_t Speed_D;

    //欧拉角
    short_union pitch;
    short_union roll;
    short_union yaw;


} send_data;

void UART_DMA_Init(void);
void UART_DMA_Start_tx(uint8_t size);
void UART_DMA_Read(void);
int8_t UART_data_check(uint8_t	*pdata);//接收数据分析
void UART_data_send(send_data *data);

#endif

