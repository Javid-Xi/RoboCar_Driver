/**
************************************************************
* @file         main.c
* @brief        ���̿���
* @author       Javid
* @date         2020-05-08
* @version      1.0
*
***********************************************************/

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "uart.h"
#include "uart_mpu.h"
#include "motor.h"
#include "encoder.h"
#include "tim.h"
#include "ps2.h"
#include "key.h"
#include "adc.h"
#include "pid.h"

//��������
#define ENCODER_MID_VALUE  30000  //�������м�ֵ
#define VBAT_MIN    1050  //3S﮵�ص͵�ѹ����ֵ10.5V

//�������
#define ROBOT_AB  1   //����ߴ磬���־������ľ�֮��

#define LIMIT_VX  50  //�ٶ�����
#define LIMIT_VY  50  //�ٶ�����
#define LIMIT_VZ  50  //�ٶ�����

//���������ƣ�0-A��1-B��2-C��3-D
int16_t encoder[4];	//����������ֵ
int16_t encoder_delta[4];	//��������Ա仯ֵ,����ʵ���ٶ�
int16_t encoder_delta_target[4] = {0}; //������Ŀ��ֵ������Ŀ���ٶ�
int16_t motor_pwm[4];  //���PWM�ٶ�

//�����ٶ�
int8_t vx; //X���˶��ٶȣ����ƺ����ƶ�
int8_t vy; //Y���˶��ٶȣ�����ǰ���ƶ�
int8_t vz; //Z���˶��ٶȣ�����ת��

rcv_data	uart_rcv_data;//���ݽ���
send_data uart_send_data;//���ݷ���
extern MPU_rcv_data uart_mpu_rcv_data;

//���ܺ���
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz); //����ѧ����

void UART_data_analyze(uint8_t *comdata);	//�����������ݽ���
void PS2_data_analyze(void);		//PS2�������ݽ���


/*************************************************
* Function: main
* Description: ������
*************************************************/
int main(void)
{
    uint8_t cnt = 1;  //���ڼ�������
	
    //�����ж����ȼ�����
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //�����ʼ��
    MOTOR_Init(10);

    //�����ʱ��ʼ��
    delay_init();

    //JTAG������
    JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
    JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���

    //LED�ƣ�ps2�ֱ���ʼ��
    KEY_Init();
    LED_Init();
    PS2_Init();

    //���ڳ�ʼ��
    UART_DMA_Init();
	UART_MPU_DMA_Init();
	
	//ADC��ʼ��
    ADC_DMA_Init();

    //��ʱ����ʼ��
    TIM6_Init(10000);//���ö�ʱ�����ڶ�ʱʱ�䣬10ms
    TIM6_Cmd(ENABLE);//��ʱ��ʹ��

    //��������ʼ��
    ENCODER_AB_Init(ENCODER_MID_VALUE * 2); //������������ʼ��
    ENCODER_CD_Init(ENCODER_MID_VALUE * 2); //������������ʼ��
    ENCODER_EF_Init(ENCODER_MID_VALUE * 2); //������������ʼ��
    ENCODER_GH_Init(ENCODER_MID_VALUE * 2); //������������ʼ��

    //���ñ�������ʼֵ
    ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
    ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
    ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
    ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

    delay_ms(500);

    while (1)
    {
        //ִ�����ڣ�10ms��100Hz
        if(TIM_CheckIrqStatus())
        {
            //����������仯ֵ������ȡС��ʵ���ٶ�
            encoder_delta[0] = (ENCODER_AB_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[1] = -(ENCODER_CD_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[2] = -(ENCODER_EF_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[3] = (ENCODER_GH_GetCounter() - ENCODER_MID_VALUE);

            //printf("%d %d %d %d\r\n", encoder_delta[0], encoder_delta[1], encoder_delta[2], encoder_delta[3]);
            //���ñ�������ʼ�м�ֵ
            ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
            ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
            ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
            ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

			//����
			if(cnt % 20 == 0)
			{
            uart_send_data.Speed_A =  encoder_delta[0];
            uart_send_data.Speed_B =  encoder_delta[1];
            uart_send_data.Speed_C =  encoder_delta[2];
            uart_send_data.Speed_D =  encoder_delta[3];
			
			uart_send_data.yaw.sv = uart_mpu_rcv_data.yaw.sv;

            UART_data_send(&uart_send_data);
			}
            //ң�ؿ���
            if(!PS2_RedLight() && cnt % 20 == 0 && Mode_get() == 0)
            {
                PS2_DataKey();	 //�ֱ�����������

                vx = 0.1 * (PS2_AnologData(5) - 0x80);
                vy = 0.2 * (- PS2_AnologData(6) + 0x7f);
                vz = 0.1 * (- PS2_AnologData(3) + 0x80);
            }

            //���ڿ���
            if(Mode_get() == 1)
            {
                vx = 0.2 * uart_rcv_data.vx;
                vy = 0.3 * uart_rcv_data.vy;
                vz = - 0.2 * uart_rcv_data.vw;
            }
			
            //�˶�����
            MOVE_Kinematics(vx, vy, vz);

//            //����ʹ��
//            if(cnt % 50 == 0)
//            {
//                printf("�ֱ�ԭʼ���ݣ�%d %d %d %d %d %d %d %d %d \r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7], Data[8]);
//                printf("�ٶȣ�%d %d %d \r\n", vx, vy, vz);
//                printf("�����ٶȣ�%d %d %d %d \r\n", encoder_delta_target[0], encoder_delta_target[1], encoder_delta_target[2], encoder_delta_target[3]);
//                printf("mode: %d \r\n", Mode_get());
//                printf("power: %d \r\n ", ADC_Get_power());
//                printf("uart_rcv_data: %d %d %d \r\n", uart_rcv_data.vx, uart_rcv_data.vy, uart_rcv_data.vw);
//            }

            //PID����
            motor_pwm[0] = Motor_PidCtl_A(encoder_delta_target[0], encoder_delta[0]);
            motor_pwm[1] = Motor_PidCtl_B(encoder_delta_target[1], encoder_delta[1]);
            motor_pwm[2] = Motor_PidCtl_C(encoder_delta_target[2], encoder_delta[2]);
            motor_pwm[3] = Motor_PidCtl_D(encoder_delta_target[3], encoder_delta[3]);

            //���ִ�ж���
            MOTOR_A_SetSpeed(motor_pwm[0]);
            MOTOR_B_SetSpeed(motor_pwm[1]);
            MOTOR_C_SetSpeed(motor_pwm[2]);
            MOTOR_D_SetSpeed(motor_pwm[3]);

            //ִ�����ڣ�100ms��10HZ
            if(cnt % 10 == 0)
            {
                //��������ʱ����ɫLED��˸
                LED_G_Toggle();
            }

            //���¼�������
            if(cnt != 100)
                cnt++;
            else {
                if(ADC_Get_power() < VBAT_MIN)
                    GPIO_ResetBits(GPIOB, GPIO_Pin_10);//�͵��� ����������
                else GPIO_SetBits(GPIOB, GPIO_Pin_10);
                cnt = 1;
            }

        }
    }
}

/*************************************************
* Function: MOVE_Kinematics
* Description: ������XYZ�ٶȽ���Ϊ���Ŀ��ת��
* Parameter: vx��vy��vz  ���������ٶ�
* Return: none
*************************************************/
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz)
{
    //�ٶ�����
    if(vx > LIMIT_VX) vx = LIMIT_VX;
    else if(vx < -LIMIT_VX) vx = -LIMIT_VX;

    if(vy > LIMIT_VY) vy = LIMIT_VY;
    else if(vy < -LIMIT_VY)	vy = -LIMIT_VY;

    if(vz > LIMIT_VZ) vz = LIMIT_VZ;
    else if(vz < -LIMIT_VZ) vz = -LIMIT_VZ;

    //�˶�����
    encoder_delta_target[0] = (-vx + vy + ROBOT_AB * vz);
    encoder_delta_target[1] = (vx + vy - ROBOT_AB * vz);
    encoder_delta_target[2] =  (-vx + vy - ROBOT_AB * vz);
    encoder_delta_target[3] = (vx + vy + ROBOT_AB * vz);
}

/*************************************************
* Function: UART_data_analyze
* Description: �������ݽ���
* Parameter: comdata ����ͨ������
* Return: none
*************************************************/
void UART_data_analyze(uint8_t *comdata)
{
    //ҡ�˿���ģʽ
    if(comdata[0] == 0x31)
    {
        //ң��ֵ�����ٶ�ֵ,ͨ��ϵ������ȡֵ��Χ
        vx = 0.5 * (int8_t)comdata[3];
        vy = 0.5 * (int8_t)comdata[4];
        vz = -0.3 * (int8_t)comdata[1];
    }

    //��п���ģʽ
    else if(comdata[0] == 0x33)
    {
        //ң��ֵ�����ٶ�ֵ,ͨ��ϵ������ȡֵ��Χ
        vx = 0;
        vy = -(int8_t)comdata[3];
        vz = -0.7 * (int8_t)comdata[2];
    }

    //��������
    else
    {
        //���õ��PID������Ĭ��
        if(comdata[0] == 11)
        {
//            motor_kp = (int16_t)((comdata[1] << 8) | comdata[2]);
//            motor_ki = (int16_t)((comdata[3] << 8) | comdata[4]);
//            motor_kd = (int16_t)((comdata[5] << 8) | comdata[6]);
        }
    }
}

/*************************************************
* Function: UART_data_analyze
* Description: PS2�����ֱ��������ݽ���
* Parameter: comdata ͨ������
* Return: none
*************************************************/
void PS2_data_analyze(void)
{
    //�ж��Ƿ�Ϊ���ģʽ��ģ��ģʽ��

    //����x,y,z���ٶ�
    PS2_DataKey();	 //�ֱ�����������
    vx = (PS2_AnologData(PSS_RX) - 0x80);
    vy = (PS2_AnologData(PSS_LY) - 0x7f);
    vz = (PS2_AnologData(PSS_LX) - 0x80);

}
