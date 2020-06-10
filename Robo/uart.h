/**
************************************************************
* @file         uart.h
* @brief        串口DMA通信
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
* @Note 		DMA_FLAG_GLx     通道 x 全局标志位 
*				DMA_FLAG_TCx     通道 x 传输完成标志位 
*				DMA_FLAG_HTx     通道 x 传输过半标志位 
*				DMA_FLAG_TEx     通道 x 传输错误标志位 
*
*				DMA_IT_GLx	     通道 x 全局中断 
*				DMA_IT_TCx	     通道 x 传输完成中断 
*				DMA_IT_HTx       通道 x 传输过半中断 
*				DMA_IT_TEx       通道 x 传输错误中断
***********************************************************/

#ifndef __UART_H
#define __UART_H

#include "sys.h"

#define USARTzTxBufferSize   32
#define USARTz							USART3
#define USARTz_GPIO						GPIOB
#define USARTz_CLK						RCC_APB1Periph_USART3
#define USARTz_GPIO_CLK					RCC_APB2Periph_GPIOB
#define USARTz_RxPin					GPIO_Pin_11
#define USARTz_TxPin					GPIO_Pin_10
#define USARTz_IRQn						USART3_IRQn
#define USARTz_DR_Base					(uint32_t)(&USART3->DR)

#define USARTz_Tx_DMA_Channe			DMA1_Channel2
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL2
#define UASRTz_TX_DMA_IRQ				DMA1_Channel2_IRQn
#define UASRTz_TX_DMA_TC				DMA1_FLAG_TC2

#define USARTz_Rx_DMA_Channe			DMA1_Channel3
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL3
#define USARTz_RX_DMA_IRQ				DMA1_Channel3_IRQn

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

    short_union vx;//线速度x
    short_union vy;//线速度y
    short_union vw;//角速度

} rcv_data;

//发送数据结构
typedef	struct {

    //电机编码器读数
    short_union Speed_A;
    short_union Speed_B;
    short_union Speed_C;
    short_union Speed_D;

    //偏航角
    short_union yaw;

} send_data;

void UART_DMA_Init(void);
void UART_DMA_Start_tx(uint8_t size);
void UART_DMA_Read(void);
int8_t UART_data_check(const uint8_t *pdata);//接收数据分析
void UART_data_send(const send_data *data);//数据包发送

#endif

