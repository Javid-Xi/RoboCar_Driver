/**
****************************************************************************
* @file         ps2.c
* @brief        ps2ң��������
* @author       Javid
* @date         2020-05-14
* @version      1.0
* @Note			PS2���ݶ���
*   			BYTE   DATA   ����
*  				01     idle
*   			02     0x73   �ֱ��Ĺ���ģʽ
*   			03     0x5A   Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7
*   			04     data   SLCT  JOYR  JOYL  STRT   UP   RGIHT  DOWN   L
*   			05     data   L2     R2     L1    R1   /\     O     X    ��
*   			06     data   �ұ�ҡ��  0x00 = ��    0xff = ��
*   			07     data   �ұ�ҡ��  0x00 = ��    0xff = ��
*   			08     data   ���ҡ��  0x00 = ��    0xff = ��
*   			09     data   ���ҡ��  0x00 = ��    0xff = ��
***************************************************************************/

#include "ps2.h"
#include "delay.h"

u16 Handkey;
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//����ֵ�밴����

/*************************************************
* Function: PS2_Init
* Description: PS2�ֱ���ʼ��
* Parameter: none
* Return: none
*************************************************/
void PS2_Init(void)
{ 	 											  
	GPIO_InitTypeDef GPIO_InitStructure;         
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_StructInit(&GPIO_InitStructure);	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/*************************************************
* Function: PS2_Cmd
* Description: ���ֱ���������
* Parameter: cmd
* Return: none
*************************************************/
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //�����Ϊ����λ
		}
		else DO_L;

		CLK_H;                        //ʱ������
		delay_us(50);
		CLK_L;
		delay_us(50);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
	}
}

/*************************************************
* Function: PS2_RedLight
* Description: �ж��Ƿ�Ϊ���ģʽ
* Parameter: none
* Return: 0,���ģʽ; 1,����ģʽ;
*************************************************/
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}

/*************************************************
* Function: PS2_ReadData
* Description: ��ȡ�ֱ�����
* Parameter: none
* Return: none
*************************************************/
void PS2_ReadData(void)
{
	volatile u8 byte=0;
	volatile u16 ref=0x01;

	CS_L;

	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������

	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{
			CLK_H;
			CLK_L;
			delay_us(50);
			CLK_H;
		      if(DI)
		      Data[byte] = ref|Data[byte];
		}
        delay_us(50);
	}
	CS_H;	
}

/*************************************************
* Function: PS2_DataKey
* Description: �Զ�������PS2�����ݽ��д���  ֻ�����˰�������
* Parameter: none
* Return: ���µİ���ֵ
* Note: Ĭ�������Ǻ��ģʽ  ֻ��һ����������ʱ,
*		0,û�а���;1,����
*************************************************/
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();

	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0,δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

/*************************************************
* Function: PS2_AnologData
* Description: �õ�һ��ҡ�˵�ģ����	��Χ0~256
* Parameter: button
* Return: ��Ӧ��������
*************************************************/
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

/*************************************************
* Function: PS2_ClearData
* Description: ������ݻ�����
* Parameter: none
* Return: none
*************************************************/
void PS2_ClearData(void)
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}






