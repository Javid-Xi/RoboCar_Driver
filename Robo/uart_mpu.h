/**
************************************************************
* @file         uart_mpu.h
* @brief        串口2DMA通信 读取jy-901模块姿态数据
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __UART_MPU_H
#define __UART_MPU_H

#include "sys.h"
#include "uart.h"

#define UART_MPUTxBufferSize   64
#define UART_MPU							USART2
#define UART_MPU_GPIO						GPIOA
#define UART_MPU_CLK						RCC_APB1Periph_USART2
#define UART_MPU_GPIO_CLK					RCC_APB2Periph_GPIOA
#define UART_MPU_RxPin						GPIO_Pin_3
#define UART_MPU_TxPin						GPIO_Pin_2
#define UART_MPU_IRQn						USART2_IRQn
#define UART_MPU_DR_Base					(uint32_t)(&USART2->DR)

#define UART_MPU_Tx_DMA_Channe				DMA1_Channel7
#define UART_MPU_Tx_DMA_FLAG				DMA1_FLAG_GL7
#define UART_MPU_TX_DMA_IRQ					DMA1_Channel7_IRQn
#define UART_MPU_TX_DMA_TC					DMA1_FLAG_TC7

#define UART_MPU_Rx_DMA_Channe				DMA1_Channel6
#define UART_MPU_Rx_DMA_FLAG				DMA1_FLAG_GL6
#define UART_MPU_RX_DMA_IRQ					DMA1_Channel6_IRQn


//接收数据结构
typedef	struct {

    //欧拉角
    short_union pitch;
    short_union roll;
    short_union yaw;

    //温度
    short_union temp;

    //四元数
    short_union q0;
    short_union q1;
    short_union q2;
    short_union q3;

} MPU_rcv_data;

void UART_MPU_DMA_Init(void);
void UART_MPU_DMA_Start(uint8_t size);
void UART_MPU_DMA_Read(void);

int8_t UART_MPU_data_check(const uint8_t *pdata);//接收数据分析
void UART_MPU_data_send(u8 mode);

#endif

