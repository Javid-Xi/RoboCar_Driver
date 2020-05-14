/**
************************************************************
* @file         uart.c
* @brief        ����DMAͨ��
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#ifndef __UART_BT_H
#define __UART_BT_H

#include "sys.h"

//#define		DINT()		__disable_irq()
//#define		EINT()		__enable_irq()
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
#define UASRTz_RX_DMA_IRQ				DMA1_Channel5_IRQn

//��������HEX���ٻ�ȡ
typedef	union {
    float fv;
    uint8_t cv[4];
} float_union;

//�������ݽṹ
typedef	struct {

    int8_t		vx;//���ٶ�x
    int8_t		vy;//���ٶ�y
    int8_t		vw;//���ٶ�

} rcv_data;

//�������ݽṹ
typedef	struct {

    float_union	x_pos;//x��������
    float_union	y_pos;//y��������
    float_union	x_v;//x�����ٶ�
    float_union	y_v;//y�����ٶ�
    float_union	angular_v;//���ٶ�
    float_union	pose_angular;//�Ƕ�

} send_data;

void UART_Init(void);
void UART_DMA_Start_tx(uint8_t size);
void UART_DMA_Read(void);
int8_t UART_data_check(uint8_t	*pdata);//�������ݷ���
void UART_data_send(void);//���ݴ��������

#endif

