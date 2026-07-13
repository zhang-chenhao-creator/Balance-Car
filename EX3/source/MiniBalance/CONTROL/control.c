#include "control.h"

/*
 * control.c 是本工程最核心的文件。
 *
 * 控制链路可以概括成一句话：
 *   MPU6050 给出车身角度和角速度，编码器给出轮子运动趋势，
 *   角度环负责“不要倒”，速度环负责“不要一直跑”，最后合成为左右电机 PWM。
 *
 * 本实验只做直立平衡，所以左右电机采用同一个控制量：
 *   Motor_Left  = Balance_Pwm + Velocity_Pwm
 *   Motor_Right = Balance_Pwm + Velocity_Pwm
 *
 * 如果以后扩展转向环，才需要在左右轮之间加入差速项。
 */
short Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
float Velocity_Left = 0, Velocity_Right = 0;

int EXTI9_5_IRQHandler(void)
{
    static int Voltage_Temp, Voltage_Count, Voltage_All;
    static u8 Flag_Target;
    int Balance_Pwm, Velocity_Pwm;

    if (INT == 0)
    {
        /*
         * MPU6050 的 INT 引脚接到 PB9，对应 EXTI9_5_IRQn。
         * 每次 MPU 数据准备好后进入本中断，因此这里就是闭环控制的固定周期入口。
         */
        EXTI->PR = 1 << 9;

        /*
         * 1. 读取左右编码器。
         * 右轮取负号是为了统一左右轮“向前”为同一符号，后面的速度环才能直接相加。
         */
        Encoder_Left = Read_Encoder(4);
        Encoder_Right = -Read_Encoder(8);

        /*
         * 2. 降低部分慢任务频率。
         * 姿态和电机控制每次中断都算，电池电压等慢变量隔一次处理，减少中断时间。
         */
        Flag_Target = !Flag_Target;

        /* 3. 姿态解算：得到 Angle_Balance 和 Gyro_Balance。 */
        Get_Angle(Way_Angle);

        /* 4. 编码器换算为线速度，主要用于状态观察和后续扩展。 */
        Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);

        /* 与主循环配合的 50ms 级软件节拍。 */
        if (delay_flag == 1)
        {
            if (++delay_50 == 10)
            {
                delay_50 = 0;
                delay_flag = 0;
            }
        }

        if (Flag_Target == 1)
        {
            /* 电池电压做 100 次平均，避免瞬时电压波动导致误判低电。 */
            Voltage_Temp = Get_battery_volt();
            Voltage_Count++;
            Voltage_All += Voltage_Temp;
            if (Voltage_Count == 100)
            {
                Voltage = Voltage_All / 100;
                Voltage_All = 0;
                Voltage_Count = 0;
            }
            return 0;
        }

        /*
         * 5. 级联控制计算。
         * Balance() 是内环：角度 PD，快速抵抗倾倒；
         * Velocity() 是外环：速度 PI，慢速抑制小车持续前后跑。
         */
        Balance_Pwm = Balance(Angle_Balance, Gyro_Balance);
        Velocity_Pwm = Velocity(Encoder_Left, Encoder_Right);

        /*
         * 6. 输出合成。
         * 平衡演示不做转向，左右轮输出相同，保证车只在前后方向修正。
         */
        Motor_Left = Balance_Pwm + Velocity_Pwm;
        Motor_Right = Balance_Pwm + Velocity_Pwm;

        /* 7. PWM 限幅，防止参数过大时电机输出饱和到危险状态。 */
        Motor_Left = PWM_Limit(Motor_Left, 6900, -6900);
        Motor_Right = PWM_Limit(Motor_Right, 6900, -6900);

        /*
         * 8. 安全保护。
         * 抬起、放下、倒地、电压过低、按键停机都会影响是否允许 Set_Pwm。
         */
        if (Pick_Up(Acceleration_Z, Angle_Balance, Encoder_Left, Encoder_Right))
            Pick_up_stop = 1;
        if (Put_Down(Angle_Balance, Encoder_Left, Encoder_Right))
            Pick_up_stop = 0;
        if (Turn_Off(Angle_Balance, Voltage) == 0)
            Set_Pwm(Motor_Left, Motor_Right);
    }
    return 0;
}

int Balance(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    int balance;

    /*
     * 角度环 PD：负责让车身回到直立角度。
     *
     * Angle_bias = 目标角度 - 当前角度
     * Gyro_bias  = 目标角速度 - 当前角速度
     *
     * P 项：车身偏得越多，轮子给的纠偏力越大。
     * D 项：车身倒得越快，给的阻尼越强，用来抑制过冲和振荡。
     *
     * 参数除以 100 是为了让串口可以用整数传参，例如 27000 表示 270.00。
     */
    Angle_bias = Middle_angle - Angle;
    Gyro_bias = 0 - Gyro;
    balance = -Balance_Kp / 100 * Angle_bias - Gyro_bias * Balance_Kd / 100;
    return balance;
}

int Velocity(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias;
    static float Encoder_Integral;

    /*
     * 速度环 PI：负责让小车不要一直往前/往后跑。
     *
     * encoder_left + encoder_right 表示车整体前后运动趋势。
     * 目标速度为 0，所以误差写成 0 - 当前速度。
     */
    Encoder_Least = 0 - (encoder_left + encoder_right);

    /*
     * 一阶低通滤波：
     *   新速度误差 = 84% 旧值 + 16% 新编码器误差
     * 这样能削弱编码器瞬时毛刺，避免速度环直接把噪声放大到电机上。
     */
    Encoder_bias *= 0.84f;
    Encoder_bias += Encoder_Least * 0.16f;

    /*
     * 积分项用于消除长期偏差。
     * 如果车总是慢慢向一个方向漂，积分会逐步累积反向修正量。
     */
    Encoder_Integral += Encoder_bias;

    /* 积分限幅，防止长时间倒地或被手按住时积分过大，重新扶正后突然跑飞。 */
    if (Encoder_Integral > 380000) Encoder_Integral = 380000;
    if (Encoder_Integral < -380000) Encoder_Integral = -380000;

    /*
     * 速度 PI 输出。
     * Velocity_Kp 处理当前速度偏差，Velocity_Ki 处理累计偏差。
     */
    velocity = -Encoder_bias * Velocity_Kp / 100 - Encoder_Integral * Velocity_Ki / 100;

    /* 停机或倒地时清积分，保证下一次 ARM 从干净状态开始。 */
    if (Turn_Off(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
        Encoder_Integral = 0;
    return velocity;
}

int Turn(float gyro)
{
    return 0;
}

void Set_Pwm(int motor_left, int motor_right)
{
    /*
     * 根据 PWM 正负号选择电机 H 桥方向。
     * TIM3 的四个通道分别控制左右电机的两个输入端。
     */
    if (motor_left > 0)
    {
        PWMA_IN1 = 7200;
        PWMA_IN2 = 7200 - motor_left;
    }
    else
    {
        PWMA_IN1 = 7200 + motor_left;
        PWMA_IN2 = 7200;
    }

    if (motor_right > 0)
    {
        PWMB_IN1 = 7200 - motor_right;
        PWMB_IN2 = 7200;
    }
    else
    {
        PWMB_IN1 = 7200;
        PWMB_IN2 = 7200 + motor_right;
    }
}

int PWM_Limit(int IN, int max, int min)
{
    /* 通用限幅函数，所有电机输出在写入 PWM 前都必须经过这里。 */
    int OUT = IN;
    if (OUT > max) OUT = max;
    if (OUT < min) OUT = min;
    return OUT;
}

u8 Turn_Off(float angle, int voltage)
{
    u8 temp;

    /*
     * 总安全停机判断：
     * 1. KEY2 未允许或串口 STOP；
     * 2. 车身倾角超过 +/-40 度，认为已经倒地；
     * 3. 电池电压过低；
     * 4. 抬起保护触发。
     */
    Flag_Stop = KEY2_STATE;
    if (KEY2_STATE == 1) Pick_up_stop = 0;
    if (angle < -40 || angle > 40 || Flag_Stop == 1 || voltage < 1000 || Pick_up_stop == 1)
    {
        temp = 1;
        PWMA_IN1 = 0;
        PWMA_IN2 = 0;
        PWMB_IN1 = 0;
        PWMB_IN2 = 0;
    }
    else
    {
        temp = 0;
    }
    return temp;
}

void Get_Angle(u8 way)
{
    float gyro_x, gyro_y;
    Temperature = Read_Temperature();

    /*
     * 姿态解算入口。
     * way=1：使用 MPU6050 内部 DMP 输出 Pitch；
     * way=2：用加速度角 + 陀螺仪角速度做 Kalman 滤波；
     * way=3：用加速度角 + 陀螺仪角速度做互补滤波。
     */
    if (way == 1)
    {
        Read_DMP();
        Angle_Balance = Pitch;
        Gyro_Balance = gyro[0];
        Gyro_Turn = gyro[2];
        Acceleration_Z = accel[2];
    }
    else
    {
        /* 直接读取 MPU6050 原始陀螺仪和加速度计寄存器。 */
        Gyro_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L);
        Gyro_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L);
        Gyro_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L);
        Accel_X = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_L);
        Accel_Y = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_L);
        Accel_Z = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) + I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_L);

        /*
         * 加速度计通过 atan2 得到静态倾角，优点是长期不漂移，缺点是动态时受震动影响；
         * 陀螺仪给出角速度，短时间响应快，缺点是积分后会漂移。
         * Kalman/互补滤波的目的就是把两者优势合起来。
         */
        Gyro_Balance = -Gyro_X;
        Accel_Angle_x = atan2(Accel_Y, Accel_Z) * 180 / PI;
        Accel_Angle_y = atan2(Accel_X, Accel_Z) * 180 / PI;
        gyro_x = Gyro_X / 16.4f;
        gyro_y = Gyro_Y / 16.4f;

        if (Way_Angle == 2)
        {
            Pitch = -Kalman_Filter_x(Accel_Angle_x, gyro_x);
            Roll = -Kalman_Filter_y(Accel_Angle_y, gyro_y);
        }
        else if (Way_Angle == 3)
        {
            Pitch = -Complementary_Filter_x(Accel_Angle_x, gyro_x);
            Roll = -Complementary_Filter_y(Accel_Angle_y, gyro_y);
        }

        Angle_Balance = Pitch;
        Gyro_Turn = Gyro_Z;
        Acceleration_Z = Accel_Z;
    }
}

int myabs(int a)
{
    return (a < 0) ? -a : a;
}

int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right)
{
    static u16 flag, count0, count1, count2;

    /*
     * 抬起检测是三段式状态机：
     * 1. 先判断轮子基本不动；
     * 2. 再判断 Z 轴加速度明显变大且角度仍在直立附近；
     * 3. 最后如果编码器突然高速变化，认为车被抬起，关闭输出。
     */
    if (flag == 0)
    {
        if (myabs(encoder_left) + myabs(encoder_right) < 150) count0++;
        else count0 = 0;
        if (count0 > 10) flag = 1, count0 = 0;
    }
    if (flag == 1)
    {
        if (++count1 > 200) count1 = 0, flag = 0;
        if (Acceleration > 30000 && Angle > (-20 + Middle_angle) && Angle < (20 + Middle_angle)) flag = 2;
    }
    if (flag == 2)
    {
        if (++count2 > 100) count2 = 0, flag = 0;
        if (myabs(encoder_left + encoder_right) > 3000)
        {
            flag = 0;
            return 1;
        }
    }
    return 0;
}

int Put_Down(float Angle, int encoder_left, int encoder_right)
{
    static u16 flag, count;

    /*
     * 放下检测：抬起保护后，只有车身回到直立附近且轮子低速接触地面，
     * 才允许清除 Pick_up_stop，避免手持状态下误启动。
     */
    if (Pick_up_stop == 0) return 0;
    if (flag == 0)
    {
        if (Angle > (-10 + Middle_angle) && Angle < (10 + Middle_angle) && encoder_left == 0 && encoder_right == 0)
            flag = 1;
    }
    if (flag == 1)
    {
        if (++count > 50) count = 0, flag = 0;
        if (encoder_left > 3 && encoder_right > 3 && encoder_left < 100 && encoder_right < 100)
        {
            flag = 0;
            return 1;
        }
    }
    return 0;
}

void Get_Velocity_Form_Encoder(int encoder_left, int encoder_right)
{
    float Rotation_Speed_L, Rotation_Speed_R;

    /*
     * 编码器计数换算为线速度：
     * 编码器增量 -> 电机转速 -> 轮子转速 -> 轮子线速度。
     * 当前平衡控制主要使用 encoder_left/right 原始增量，线速度变量用于观察和扩展。
     */
    Rotation_Speed_L = encoder_left * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Left = Rotation_Speed_L * PI * Diameter_67;
    Rotation_Speed_R = encoder_right * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Right = Rotation_Speed_R * PI * Diameter_67;
}
