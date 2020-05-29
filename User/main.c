/**
************************************************************
* @file         main.c
* @brief        底盘控制
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

//常量定义
#define ENCODER_MID_VALUE  30000  //编码器中间值
#define VBAT_MIN    1050  //3S锂电池低电压报警值10.5V

//车体参数
#define ROBOT_AB  1   //车体尺寸，车轮距离中心距之和

#define LIMIT_VX  200  //速度限制
#define LIMIT_VY  200  //速度限制
#define LIMIT_VZ  200  //速度限制

//编码器控制，0-A，1-B，2-C，3-D
int16_t encoder[4];	//编码器绝对值
int16_t encoder_delta[4];	//编码器相对变化值,代表实际速度
int16_t encoder_delta_target[4] = {0}; //编码器目标值，代表目标速度
int16_t motor_pwm[4];  //电机PWM速度

//设置速度
int16_t vx; //X轴运动速度，控制横向移动
int16_t vy; //Y轴运动速度，控制前后移动
int16_t vz; //Z轴运动速度，控制转向

rcv_data	uart_rcv_data;//数据接收
send_data uart_send_data;//数据发送
extern MPU_rcv_data uart_mpu_rcv_data;

//功能函数
void Chassis_status_send(void);//发送底盘状态
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz); //逆运动模型解算
void UART_data_analyze(void);	//串口数据解析
void PS2_data_analyze(void);	//PS2控制数据解析


/*************************************************
* Function: main
* Description: 主函数
*************************************************/
int main(void)
{
    uint8_t cnt = 1;  //周期计数变量

    //设置中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //电机初始化
    MOTOR_Init(10);

    //软件延时初始化
    delay_init();

    //JTAG口设置
    JTAG_Set(JTAG_SWD_DISABLE);     //关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //打开SWD接口 可以利用主板的SWD接口调试

    //LED灯，ps2手柄初始化
    KEY_Init();
    LED_Init();
    PS2_Init();

    //串口初始化
    UART_DMA_Init();
    UART_MPU_DMA_Init();

    //ADC初始化
    ADC_DMA_Init();

    //定时器初始化
    TIM6_Init(50000);//设置定时器周期定时时间，50ms 20hz
    TIM6_Cmd(ENABLE);//定时器使能

    //编码器初始化
    ENCODER_AB_Init(ENCODER_MID_VALUE * 2); //正交编码器初始化
    ENCODER_CD_Init(ENCODER_MID_VALUE * 2); //正交编码器初始化
    ENCODER_EF_Init(ENCODER_MID_VALUE * 2); //正交编码器初始化
    ENCODER_GH_Init(ENCODER_MID_VALUE * 2); //正交编码器初始化

    //设置编码器初始值
    ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
    ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
    ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
    ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

    //系统初始化完成
    Sysinit_Complete();

    while (1)
    {
        //执行周期（50ms）20Hz
        if(TIM_CheckIrqStatus())
        {

            /************* 获取电机转速 **************/
            encoder_delta[0] = (ENCODER_AB_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[1] = -(ENCODER_CD_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[2] = -(ENCODER_EF_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[3] = (ENCODER_GH_GetCounter() - ENCODER_MID_VALUE);

            //设置编码器初始中间值
            ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
            ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
            ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
            ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

            /************* 发送底盘状态 **************/
            Chassis_status_send();

            /************* 控制模式 **************/
            //遥控控制
            if(Mode_get() == 0)
                PS2_data_analyze();
            //串口控制
            else UART_data_analyze();

            /************* 运动解算 **************/
            MOVE_Kinematics(vx, vy, vz);

//            //调试使用
//            if(cnt % 10 == 0)
//            {
//                printf("手柄原始数据：%d %d %d %d %d %d %d %d %d \r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7], Data[8]);
//                printf("速度：%d %d %d \r\n", vx, vy, vz);
//                printf("轮子实际速度：%d %d %d %d \r\n", encoder_delta[0], encoder_delta[1], encoder_delta[2], encoder_delta[3]);
//                printf("解算速度：%d %d %d %d \r\n", encoder_delta_target[0], encoder_delta_target[1], encoder_delta_target[2], encoder_delta_target[3]);
//                printf("mode: %d \r\n", Mode_get());
//                printf("power: %d \r\n ", ADC_Get_power());
//                printf("uart_rcv_data: %d %d %d \r\n", uart_rcv_data.vx.sv, uart_rcv_data.vy.sv, uart_rcv_data.vw.sv);
//            }

            /************* PID控制 **************/
            motor_pwm[0] = Motor_PidCtl_A(encoder_delta_target[0], encoder_delta[0]);
            motor_pwm[1] = Motor_PidCtl_B(encoder_delta_target[1], encoder_delta[1]);
            motor_pwm[2] = Motor_PidCtl_C(encoder_delta_target[2], encoder_delta[2]);
            motor_pwm[3] = Motor_PidCtl_D(encoder_delta_target[3], encoder_delta[3]);

            /************* 电机输出 **************/
            MOTOR_A_SetSpeed(motor_pwm[0]);
            MOTOR_B_SetSpeed(motor_pwm[1]);
            MOTOR_C_SetSpeed(motor_pwm[2]);
            MOTOR_D_SetSpeed(motor_pwm[3]);

            //执行周期（500ms）10HZ
            if(cnt % 10 == 0)
                LED_G_Toggle();


            //更新计数周期 5s
            if(cnt != 100)
                cnt++;
            else {
                if(ADC_Get_power() < VBAT_MIN)
                    GPIO_ResetBits(BEEP_PIN_Port, BEEP_PIN);//低电量 蜂鸣器报警
                else GPIO_SetBits(BEEP_PIN_Port, BEEP_PIN);
                cnt = 1;
            }
        }
    }
}

/*************************************************
* Function: MOVE_Kinematics
* Description: 由坐标XYZ速度解析为电机目标转速
* Parameter: vx，vy，vz  三轴坐标速度
* Return: none
*************************************************/
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz)
{
    //速度限制
    if(vx > LIMIT_VX) vx = LIMIT_VX;
    else if(vx < -LIMIT_VX) vx = -LIMIT_VX;

    if(vy > LIMIT_VY) vy = LIMIT_VY;
    else if(vy < -LIMIT_VY)	vy = -LIMIT_VY;

    if(vz > LIMIT_VZ) vz = LIMIT_VZ;
    else if(vz < -LIMIT_VZ) vz = -LIMIT_VZ;

    //运动解析
    encoder_delta_target[0] = (-vx + vy + ROBOT_AB * vz);
    encoder_delta_target[1] = (vx + vy - ROBOT_AB * vz);
    encoder_delta_target[2] =  (-vx + vy - ROBOT_AB * vz);
    encoder_delta_target[3] = (vx + vy + ROBOT_AB * vz);
}

/*************************************************
* Function: UART_data_analyze
* Description: 串口数据解析
* Parameter: comdata 串口通信数据
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
* Description: PS2无线手柄控制数据解析
* Parameter: none
* Return: none
*************************************************/
void PS2_data_analyze(void)
{
    if(!PS2_RedLight())
    {

        PS2_DataKey();	 //手柄按键捕获处理

        vx = 0.5 * (PS2_AnologData(5) - 0x80);
        vy = 1 * (- PS2_AnologData(6) + 0x7f);
        vz = 0.5 * (- PS2_AnologData(3) + 0x80);
    }
}

/*************************************************
* Function: Chassis_status_send
* Description: 发送底盘状态
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

