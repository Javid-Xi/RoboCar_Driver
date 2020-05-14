/**
************************************************************
* @file         sys.c
* @brief        �����������
* @author       Javid
* @date         2020-05-08
* @version      1.0
* @Note			JTAG SWD����
*				printf����
*				λ��������
***********************************************************/

#include "sys.h"

/*************************************************
* Function: JTAG_Set
* Description: JTAGģʽ����	
* Parameter:	mode:jtag,swdģʽ����;
				00,ȫʹ��;
				01,ʹ��SWD;
				10,ȫ�ر�;
				JTAG_SWD_DISABLE   0X02
				SWD_ENABLE         0X01
				JTAG_SWD_ENABLE    0X00	
* Return: none
*************************************************/
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 


/***************** ֧��printf ****************/
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
    int handle;

};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
    x = x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
    while((USART1->SR & 0X40) == 0); //ѭ������,ֱ���������
    USART1->DR = (u8) ch;
    return ch;
}
#endif
