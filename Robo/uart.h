/**
************************************************************
* @file         uart.h
* @brief        ����DMAͨ��
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
* @Note 		DMA_FLAG_GLx     ͨ�� x ȫ�ֱ�־λ 
*				DMA_FLAG_TCx     ͨ�� x ������ɱ�־λ 
*				DMA_FLAG_HTx     ͨ�� x ��������־λ 
*				DMA_FLAG_TEx     ͨ�� x ��������־λ 
*
*				DMA_IT_GLx	     ͨ�� x ȫ���ж� 
*				DMA_IT_TCx	     ͨ�� x ��������ж� 
*				DMA_IT_HTx       ͨ�� x ��������ж� 
*				DMA_IT_TEx       ͨ�� x ��������ж�
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

//��������HEX���ٻ�ȡ
typedef	union {
    float fv;
    uint8_t cv[4];
} float_union;

//short��HEX���ٻ�ȡ
typedef	union {
    short sv;
    uint8_t cv[2];
} short_union;


//�������ݽṹ
typedef	struct {

    short_union vx;//���ٶ�x
    short_union vy;//���ٶ�y
    short_union vw;//���ٶ�

} rcv_data;

//�������ݽṹ
typedef	struct {

    //�������������
    short_union Speed_A;
    short_union Speed_B;
    short_union Speed_C;
    short_union Speed_D;

    //ƫ����
    short_union yaw;

} send_data;

void UART_DMA_Init(void);
void UART_DMA_Start_tx(uint8_t size);
void UART_DMA_Read(void);
int8_t UART_data_check(const uint8_t *pdata);//�������ݷ���
void UART_data_send(const send_data *data);//���ݰ�����

#endif

