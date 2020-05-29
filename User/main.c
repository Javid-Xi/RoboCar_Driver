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

#define LIMIT_VX  200  //�ٶ�����
#define LIMIT_VY  200  //�ٶ�����
#define LIMIT_VZ  200  //�ٶ�����

//���������ƣ�0-A��1-B��2-C��3-D
int16_t encoder[4];	//����������ֵ
int16_t encoder_delta[4];	//��������Ա仯ֵ,����ʵ���ٶ�
int16_t encoder_delta_target[4] = {0}; //������Ŀ��ֵ������Ŀ���ٶ�
int16_t motor_pwm[4];  //���PWM�ٶ�

//�����ٶ�
int16_t vx; //X���˶��ٶȣ����ƺ����ƶ�
int16_t vy; //Y���˶��ٶȣ�����ǰ���ƶ�
int16_t vz; //Z���˶��ٶȣ�����ת��

rcv_data	uart_rcv_data;//���ݽ���
send_data uart_send_data;//���ݷ���
extern MPU_rcv_data uart_mpu_rcv_data;

//���ܺ���
void Chassis_status_send(void);//���͵���״̬
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz); //���˶�ģ�ͽ���
void UART_data_analyze(void);	//�������ݽ���
void PS2_data_analyze(void);	//PS2�������ݽ���


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
    JTAG_Set(JTAG_SWD_DISABLE);     //�ر�JTAG�ӿ�
    JTAG_Set(SWD_ENABLE);           //��SWD�ӿ� �������������SWD�ӿڵ���

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
    TIM6_Init(50000);//���ö�ʱ�����ڶ�ʱʱ�䣬50ms 20hz
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

    //ϵͳ��ʼ�����
    Sysinit_Complete();

    while (1)
    {
        //ִ�����ڣ�50ms��20Hz
        if(TIM_CheckIrqStatus())
        {

            /************* ��ȡ���ת�� **************/
            encoder_delta[0] = (ENCODER_AB_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[1] = -(ENCODER_CD_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[2] = -(ENCODER_EF_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[3] = (ENCODER_GH_GetCounter() - ENCODER_MID_VALUE);

            //���ñ�������ʼ�м�ֵ
            ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
            ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
            ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
            ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

            /************* ���͵���״̬ **************/
            Chassis_status_send();

            /************* ����ģʽ **************/
            //ң�ؿ���
            if(Mode_get() == 0)
                PS2_data_analyze();
            //���ڿ���
            else UART_data_analyze();

            /************* �˶����� **************/
            MOVE_Kinematics(vx, vy, vz);

//            //����ʹ��
//            if(cnt % 10 == 0)
//            {
//                printf("�ֱ�ԭʼ���ݣ�%d %d %d %d %d %d %d %d %d \r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7], Data[8]);
//                printf("�ٶȣ�%d %d %d \r\n", vx, vy, vz);
//                printf("����ʵ���ٶȣ�%d %d %d %d \r\n", encoder_delta[0], encoder_delta[1], encoder_delta[2], encoder_delta[3]);
//                printf("�����ٶȣ�%d %d %d %d \r\n", encoder_delta_target[0], encoder_delta_target[1], encoder_delta_target[2], encoder_delta_target[3]);
//                printf("mode: %d \r\n", Mode_get());
//                printf("power: %d \r\n ", ADC_Get_power());
//                printf("uart_rcv_data: %d %d %d \r\n", uart_rcv_data.vx.sv, uart_rcv_data.vy.sv, uart_rcv_data.vw.sv);
//            }

            /************* PID���� **************/
            motor_pwm[0] = Motor_PidCtl_A(encoder_delta_target[0], encoder_delta[0]);
            motor_pwm[1] = Motor_PidCtl_B(encoder_delta_target[1], encoder_delta[1]);
            motor_pwm[2] = Motor_PidCtl_C(encoder_delta_target[2], encoder_delta[2]);
            motor_pwm[3] = Motor_PidCtl_D(encoder_delta_target[3], encoder_delta[3]);

            /************* ������ **************/
            MOTOR_A_SetSpeed(motor_pwm[0]);
            MOTOR_B_SetSpeed(motor_pwm[1]);
            MOTOR_C_SetSpeed(motor_pwm[2]);
            MOTOR_D_SetSpeed(motor_pwm[3]);

            //ִ�����ڣ�500ms��10HZ
            if(cnt % 10 == 0)
                LED_G_Toggle();


            //���¼������� 5s
            if(cnt != 100)
                cnt++;
            else {
                if(ADC_Get_power() < VBAT_MIN)
                    GPIO_ResetBits(BEEP_PIN_Port, BEEP_PIN);//�͵��� ����������
                else GPIO_SetBits(BEEP_PIN_Port, BEEP_PIN);
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
void UART_data_analyze(void)
{
    vx = uart_rcv_data.vx.sv;
    vy = uart_rcv_data.vy.sv;
    vz = uart_rcv_data.vw.sv;
}

/*************************************************
* Function: UART_data_analyze
* Description: PS2�����ֱ��������ݽ���
* Parameter: none
* Return: none
*************************************************/
void PS2_data_analyze(void)
{
    if(!PS2_RedLight())
    {

        PS2_DataKey();	 //�ֱ�����������

        vx = 0.5 * (PS2_AnologData(5) - 0x80);
        vy = 1 * (- PS2_AnologData(6) + 0x7f);
        vz = 0.5 * (- PS2_AnologData(3) + 0x80);
    }
}

/*************************************************
* Function: Chassis_status_send
* Description: ���͵���״̬
* Parameter: none
* Return: none
*************************************************/
void Chassis_status_send(void)
{
    uart_send_data.Speed_A.sv =  encoder_delta[0];
    uart_send_data.Speed_B.sv =  encoder_delta[1];
    uart_send_data.Speed_C.sv =  encoder_delta[2];
    uart_send_data.Speed_D.sv =  encoder_delta[3];

    uart_send_data.yaw.sv = uart_mpu_rcv_data.yaw.sv;

    UART_data_send(&uart_send_data);
}

