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
#include "motor.h"
#include "encoder.h"
#include "tim.h"
#include "ps2.h"
#include "key.h"
#include "adc.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

//常量定义
#define ENCODER_MID_VALUE  30000  //编码器中间值
#define VBAT_MIN    1050  //3S锂电池低电压报警值10.5V

//车体参数
#define ROBOT_AB  1   //车体尺寸，车轮距离中心距之和

#define LIMIT_VX  50  //速度限制
#define LIMIT_VY  50  //速度限制
#define LIMIT_VZ  50  //速度限制

#define PID_SCALE  0.01f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

//编码器控制，0-A，1-B，2-C，3-D
int16_t encoder[4];	//编码器绝对值
int16_t encoder_delta[4];	//编码器相对变化值,代表实际速度
int16_t encoder_delta_target[4] = {0}; //编码器目标值，代表目标速度
int16_t motor_pwm[4];  //电机PWM速度

int16_t vx; //X轴运动速度，控制横向移动
int16_t vy; //Y轴运动速度，控制前后移动
int16_t vz; //Z轴运动速度，控制转向

int16_t motor_kp = 130; //PID参数
int16_t motor_ki = 0; //PID参数
int16_t motor_kd = 30; //PID参数

extern rcv_data	uart_rcv_data;//数据接收

//功能函数
void MOVE_Kinematics(int16_t vx, int16_t vy, int16_t vz); //运行学解析
int16_t Motor_PidCtl_A(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_B(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_C(int16_t spd_target, int16_t spd_current);   //PID控制
int16_t Motor_PidCtl_D(int16_t spd_target, int16_t spd_current);   //PID控制
void UART_data_analyze(uint8_t *comdata);	//蓝牙控制数据解析
void PS2_data_analyze(void);		//PS2控制数据解析


/*************************************************
* Function: main
* Description: 主函数
*************************************************/
int main(void)
{
    uint8_t cnt = 1;  //周期计数变量

    float pitch, roll, yaw; 		//欧拉角
    short aacx, aacy, aacz;		//加速度传感器原始数据
    short gyrox, gyroy, gyroz;	//陀螺仪原始数据
    short temp;					//温度

    //设置中断优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    //电机初始化
    MOTOR_Init(10);

    //软件延时初始化
    delay_init();

    //JTAG口设置
    JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试

    //LED灯，ps2手柄初始化
    KEY_Init();
    LED_Init();
    PS2_Init();

    //串口初始化
    UART_Init();
    ADC_DMA_Init();

	//初始化MPU6050和DMP
    MPU_Init();					
    while(mpu_dmp_init())
        printf("MPU6050初始化失败\r\n");
    printf("MPU6050初始化成功\r\n");

    //定时器初始化
    TIM6_Init(10000);//设置定时器周期定时时间，10ms
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

    delay_ms(500);

    while (1)
    {
        //执行周期（10ms）100Hz
        if(TIM_CheckIrqStatus())
        {
            //计算编码器变化值，即获取小车实际速度
            encoder_delta[0] = (ENCODER_AB_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[1] = -(ENCODER_CD_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[2] = -(ENCODER_EF_GetCounter() - ENCODER_MID_VALUE);
            encoder_delta[3] = (ENCODER_GH_GetCounter() - ENCODER_MID_VALUE);

            //printf("%d %d %d %d\r\n", encoder_delta[0], encoder_delta[1], encoder_delta[2], encoder_delta[3]);
            //设置编码器初始中间值
            ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
            ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
            ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
            ENCODER_GH_SetCounter(ENCODER_MID_VALUE);

			//遥控控制
            if(!PS2_RedLight() && cnt % 20 == 0 && Mode_get() == 0)
            {
                PS2_DataKey();	 //手柄按键捕获处理

                vx = 0.1 * (PS2_AnologData(5) - 0x80);
                vy = 0.2 * (- PS2_AnologData(6) + 0x7f);
                vz = 0.1 * (- PS2_AnologData(3) + 0x80);
            }
			
			//串口控制
			if(Mode_get() == 1)
			{
			         vx = 0.2 * uart_rcv_data.vx;
            vy = 0.3 * uart_rcv_data.vy;
            vz = - 0.2 * uart_rcv_data.vw;
			
			}

			//运动解算
            MOVE_Kinematics(vx, vy, vz);

			//调试使用
            if(cnt % 50 == 0)
            {
                printf("手柄原始数据：%d %d %d %d %d %d %d %d %d \r\n", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7], Data[8]);
                printf("速度：%d %d %d \r\n", vx, vy, vz);
                printf("解算速度：%d %d %d %d \r\n", encoder_delta_target[0], encoder_delta_target[1], encoder_delta_target[2], encoder_delta_target[3]);
                printf("mode: %d \r\n", Mode_get());
                printf("power: %d \r\n ", ADC_Get_power());
                printf("uart_rcv_data: %d %d %d \r\n", uart_rcv_data.vx, uart_rcv_data.vy, uart_rcv_data.vw);

             while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}	//加while循环防止fifo溢出
				 
                    temp = MPU_Get_Temperature();	//得到温度值
                    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);	//得到加速度传感器数据
                    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);	//得到陀螺仪数据
                    printf("温度：%d\r\n", temp);
                    printf("加速度：%d %d %d\r\n", aacx, aacy, aacz);
                    printf("陀螺仪：%d %d %d\r\n", gyrox, gyroy, gyroz);
                    printf("角度：%f %f %f\r\n", pitch, roll, yaw);
            }

            //PID控制
            motor_pwm[0] = Motor_PidCtl_A(encoder_delta_target[0], encoder_delta[0]);
            motor_pwm[1] = Motor_PidCtl_B(encoder_delta_target[1], encoder_delta[1]);
            motor_pwm[2] = Motor_PidCtl_C(encoder_delta_target[2], encoder_delta[2]);
            motor_pwm[3] = Motor_PidCtl_D(encoder_delta_target[3], encoder_delta[3]);

            //电机执行动作
            MOTOR_A_SetSpeed(motor_pwm[0]);
            MOTOR_B_SetSpeed(motor_pwm[1]);
            MOTOR_C_SetSpeed(motor_pwm[2]);
            MOTOR_D_SetSpeed(motor_pwm[3]);

            //执行周期（100ms）10HZ
            if(cnt % 10 == 0)
            {
                //正常运行时，绿色LED闪烁
                LED_G_Toggle();
            }

            //更新计数周期
            if(cnt != 100)
                cnt++;
            else {
                if(ADC_Get_power() < VBAT_MIN)
                    GPIO_ResetBits(GPIOB, GPIO_Pin_10);//低电量 蜂鸣器报警
                else GPIO_SetBits(GPIOB, GPIO_Pin_10);
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
* Function: Motor_PidCtl_A
* Description: 	电机A PID控制函数	
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_A(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_B
* Description: 	电机B PID控制函数	
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_B(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;


    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_C
* Description: 	电机C PID控制函数	
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_C(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: Motor_PidCtl_D
* Description: 	电机D PID控制函数	
* Parameter:  	spd_target:编码器速度目标值
*				spd_current: 编码器速度当前值
* Return: 电机PWM速度
*************************************************/
int16_t Motor_PidCtl_D(int16_t spd_target, int16_t spd_current)
{
    static int16_t motor_pwm_out;
    static int32_t bias, bias_last, bias_integral = 0;

    bias = spd_target - spd_current;

    bias_integral += bias;

    if(bias_integral > PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
    if(bias_integral < -PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;

    motor_pwm_out += motor_kp * bias * PID_SCALE + motor_kd * (bias - bias_last) * PID_SCALE + motor_ki * bias_integral * PID_SCALE;

    bias_last = bias;

    return motor_pwm_out;
}

/*************************************************
* Function: UART_data_analyze
* Description: 串口数据解析
* Parameter: comdata 串口通信数据
* Return: none
*************************************************/
void UART_data_analyze(uint8_t *comdata)
{
    //摇杆控制模式
    if(comdata[0] == 0x31)
    {
        //遥控值换算速度值,通过系数控制取值范围
        vx = 0.5 * (int8_t)comdata[3];
        vy = 0.5 * (int8_t)comdata[4];
        vz = -0.3 * (int8_t)comdata[1];
    }

    //体感控制模式
    else if(comdata[0] == 0x33)
    {
        //遥控值换算速度值,通过系数控制取值范围
        vx = 0;
        vy = -(int8_t)comdata[3];
        vz = -0.7 * (int8_t)comdata[2];
    }

    //参数设置
    else
    {
        //设置电机PID参数，默认
        if(comdata[0] == 11)
        {
            motor_kp = (int16_t)((comdata[1] << 8) | comdata[2]);
            motor_ki = (int16_t)((comdata[3] << 8) | comdata[4]);
            motor_kd = (int16_t)((comdata[5] << 8) | comdata[6]);
        }
    }
}

/*************************************************
* Function: UART_data_analyze
* Description: PS2无线手柄控制数据解析
* Parameter: comdata 通信数据
* Return: none
*************************************************/
void PS2_data_analyze(void)
{
    //判断是否为红灯模式（模拟模式）

    //设置x,y,z轴速度
    PS2_DataKey();	 //手柄按键捕获处理
    vx = (PS2_AnologData(PSS_RX) - 0x80);
    vy = (PS2_AnologData(PSS_LY) - 0x7f);
    vz = (PS2_AnologData(PSS_LX) - 0x80);

}
