#include "stm32f10x.h"
#include "sys.h"

/*
 * 实验三：B585 平衡小车级联 PID 调参演示版
 *
 * 说明：
 * 1. 本工程入口文件不是 main.c，而是 Keil 工程中的 USER/MiniBalance.c。
 *    下面的 int main(void) 就是程序主入口。
 * 2. 本版本面向“平衡站立 + 串口调参”验收，保留最小闭环链路：
 *    MPU6050 姿态采集 -> 编码器速度反馈 -> 角度环 PD -> 速度环 PI -> 电机 PWM。
 * 3. 与全功能参考工程相比，本版本去掉了蓝牙 APP、OLED 菜单、超声波、遥控等
 *    展示无关模块，避免答辩时主线被外围功能干扰。
 * 4. 串口调参命令在 SYSTEM/usart/usart.c 中实现，控制公式在
 *    MiniBalance/CONTROL/control.c 中实现。
 */

/* 抬起检测标志。为 1 时控制输出会被关闭，防止车轮离地后高速空转。 */
u8 Pick_up_stop = 0;

/* 机械零点角。小车并不一定在 0 度时正好直立，所以保留 MID 命令可微调零点。 */
int Middle_angle = 0;

/* 姿态解算方式：1=DMP，2=Kalman，3=Complementary。本次默认使用 Kalman。 */
u8 Way_Angle = 2;

/* 方向/遥控相关变量在精简版中保留为兼容接口，当前演示不使用转向和遥控。 */
u16 Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0;
u16 Flag_velocity = 2, Target_Velocity = 300;
float RC_Velocity = 0, RC_Turn_Velocity = 0;

/* 总停机标志。上电默认 STOP=1，需要串口 ARM 后才允许输出 PWM。 */
u8 Flag_Stop = 1;
u16 determine = 0;

/* 编码器增量、左右电机最终 PWM、传感器状态量。 */
int Encoder_Left = 0, Encoder_Right = 0;
int Motor_Left = 0, Motor_Right = 0;
int Temperature = 0;
int Voltage = 1200;

/* 姿态控制核心反馈量：俯仰角 Angle_Balance、俯仰角速度 Gyro_Balance。 */
float Angle_Balance = 0, Gyro_Balance = 0, Gyro_Turn = 0;
u32 Distance = 0;
u8 delay_50 = 0, PID_Send = 0;
volatile u8 delay_flag = 0;
float Acceleration_Z = 0;

/*
 * 最终演示 PID 参数。
 *
 * 注意：这里的数值是“串口命令/源码显示口径”，在控制公式中会除以 100。
 * 实际等效参数：
 *   Balance_Kp = 270.00
 *   Balance_Kd = 1.10
 *   Velocity_Kp = 4.00
 *   Velocity_Ki = 0.02
 *
 * 调参逻辑：
 *   先关闭速度环，只调角度环 Kp；
 *   再固定 Kp 调角度环 Kd；
 *   角度环能抗倒后，再加入速度环 Kp；
 *   最后只加入很小的 Ki 消除长期漂移。
 */
float Balance_Kp = 27000, Balance_Kd = 110;
float Velocity_Kp = 400, Velocity_Ki = 2;
float Turn_Kp = 0, Turn_Kd = 0;

int main(void)
{
    /*
     * 初始化顺序说明：
     * 1. 先配置中断优先级、延时和调试接口；
     * 2. 再初始化基础外设：LED、按键、电机 PWM、串口；
     * 3. 然后初始化闭环反馈外设：左右编码器、ADC、电池电压、IIC、MPU6050/DMP；
     * 4. 最后打开 MPU6050 的外部中断。真正的控制运算不在 while 中，而是在
     *    control.c 的 EXTI9_5_IRQHandler 中按固定周期执行。
     */
    MY_NVIC_PriorityGroupConfig(2);
    delay_init();
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    LED_Init();
    KEY_Init();
    MiniBalance_PWM_Init(7199, 0);
    uart_init(115200);
    Encoder_Init_TIM8();
    Encoder_Init_TIM4();
    Adc_Init();
    IIC_Init();
    MPU6050_initialize();
    DMP_Init();
    MiniBalance_EXTI_Init();

    while (1)
    {
        /*
         * 主循环保持很轻，只做节拍等待。
         * 平衡车控制对周期稳定性要求高，因此把核心闭环放在 MPU 中断里，
         * 避免 while 中的串口或其他任务造成控制周期抖动。
         */
        delay_flag = 1;
        while (delay_flag);
    }
}






