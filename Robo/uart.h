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
#define USARTz							USART1
#define USARTz_GPIO						GPIOA
#define USARTz_CLK						RCC_APB2Periph_USART1
#define USARTz_GPIO_CLK					RCC_APB2Periph_GPIOA
#define USARTz_RxPin					GPIO_Pin_10
#define USARTz_TxPin					GPIO_Pin_9
#define USARTz_IRQn						USART1_IRQn
#define USARTz_DR_Base					(uint32_t)(&USART1->DR)

#define USARTz_Tx_DMA_Channe			DMA1_Channel4
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL4
#define UASRTz_TX_DMA_IRQ				DMA1_Channel4_IRQn

#define USARTz_Rx_DMA_Channe			DMA1_Channel5
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL5
#define USARTz_RX_DMA_IRQ				DMA1_Channel5_IRQn

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
	
	//pid
	uint8_t pid_set;
    short_union p;
    short_union i;
    short_union d;

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
	
	//PID
    short_union p;
    short_union i;
    short_union d;

} send_data;

void UART_DMA_Init(void);
void UART_DMA_Start_tx(uint8_t size);
void UART_DMA_Read(void);
int8_t UART_data_check(uint8_t	*pdata);//�������ݷ���
void UART_data_send(send_data *data);

#endif

