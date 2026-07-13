#include "sys.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * usart.c 是本工程的“调参接口”。
 *
 * EX4 同时保留两套通信：
 * 1. USART3 连接蓝牙模块，用单字节命令控制前进、后退、左右转；
 * 2. USART1 连接电脑 USB 串口，用 ASCII 命令做调试、查状态和临时调参。
 *
 * USART1 支持的 ASCII 命令：
 *   PID bk bd vk vi  一次设置角度环和速度环参数
 *   BAL kp kd        只设置角度环 PD
 *   VEL kp ki        只设置速度环 PI
 *   MID angle        调整机械零点
 *   MODE ...         切换姿态解算方式
 *   ARM / STOP       使能或停止电机输出
 *   STATUS           打印当前状态，便于录屏和答辩核对参数
 *
 * 这样做的目的：
 * 1. 老师能直接看到每次调参输入了什么；
 * 2. Excel 中的 8 组参数可以和串口命令一一对应；
 * 3. 蓝牙遥控和 USB 调试互不冲突，答辩时能同时展示“功能”和“参数状态”。
 */
static void ascii_cmd_handle(char *cmd);
static void ascii_puts(const char *s);
static void ascii_print_status(void);

/*
 * printf 重定向位置。
 *
 * Keil/ARMCC 默认 printf 会走半主机机制，离开调试器后可能卡死。
 * 这里用 __use_no_semihosting 关闭半主机，并重写 fputc：
 *   printf("abc") -> fputc('a') -> USART1->DR
 * 所以 printf 最终会从 USART1 TX 引脚输出到串口助手。
 */
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { x = x; }

int fputc(int ch, FILE *f)
{
    /* 等待 USART1 发送完成，再写入下一个字节。 */
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (u8)ch;
    return ch;
}

void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = USART1_TX，推挽复用输出。 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA10 = USART1_RX，浮空输入。 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 打开 USART1 接收中断：串口每收到 1 个字节都会进入 USART1_IRQHandler。 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* RXNE 表示“接收寄存器非空”，用于逐字节接收命令。 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    static u8 ascii_count = 0;
    static char ascii_buf[64];
    u8 byte;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        byte = (u8)USART_ReceiveData(USART1);

        /*
         * 命令以回车或换行结束。
         * 例如串口助手发送 "PID 27000 110 400 2\r\n"，
         * 这里累计到换行后再交给 ascii_cmd_handle 解析。
         */
        if (byte == 0x0D || byte == 0x0A)
        {
            if (ascii_count > 0)
            {
                ascii_buf[ascii_count] = 0;
                ascii_cmd_handle(ascii_buf);
                ascii_count = 0;
                memset(ascii_buf, 0, sizeof(ascii_buf));
            }
            return;
        }

        /* 只接收可见 ASCII 字符，过滤乱码和二进制帧。 */
        if (byte >= 0x20 && byte <= 0x7E)
        {
            if (ascii_count < (sizeof(ascii_buf) - 1))
            {
                ascii_buf[ascii_count++] = (char)byte;
            }
            else
            {
                ascii_count = 0;
                memset(ascii_buf, 0, sizeof(ascii_buf));
                ascii_puts("ERR,CMD_TOO_LONG\r\n");
            }
        }
    }
}

void usart1_send(u8 data)
{
    /* 单字节阻塞发送，调参命令返回 OK/STATUS 时使用。 */
    USART1->DR = data;
    while ((USART1->SR & 0x40) == 0);
}

static void ascii_puts(const char *s)
{
    /* 发送以 '\0' 结尾的字符串。 */
    while (*s)
    {
        usart1_send((u8)(*s++));
    }
}

static void ascii_print_status(void)
{
    char buf[360];

    /*
     * 状态帧用于答辩核对：
     * STOP  当前是否停机
     * KEY   KEY2 硬件使能状态，1 表示硬件按键正在禁止输出
     * PICK  抬起保护状态
     * SAFE  安全停机原因，0=允许输出，1=倒地，2=KEY2，3=STOP，4=低电压，5=抬起
     * MODE  姿态解算方式
     * BK/BD 角度环 Kp/Kd
     * VK/VI 速度环 Kp/Ki
     * V     电池电压原始值，约等于实际电压 * 100
     * BT    当前蓝牙运动命令
     * ANGLE/GYRO 当前姿态反馈
     * DIST/UOK/OBS 超声波距离、数据有效标志、障碍物阈值标志
     * UAUTO/UACT 超声波自动避障开关与当前动作
     * EL/ER 左右编码器
     * ML/MR 左右电机 PWM
     */
    sprintf(buf,
            "STATUS,STOP=%d,KEY=%d,PICK=%d,SAFE=%d,MODE=%d,BK=%d,BD=%d,VK=%d,VI=%d,MID=%d,V=%d,BT=%d,ANGLE=%d,GYRO=%d,DIST=%lu,UOK=%d,OBS=%d,UGUARD=%d,USTATE=%d,MISS=%d,USTOP=%d,USLOW=%d,VREQ=%d,VSAFE=%d,EL=%d,ER=%d,ML=%d,MR=%d\r\n",
            Flag_Stop, KEY2_STATE, Pick_up_stop, Safety_Stop_Reason,
            (int)Way_Angle, (int)Balance_Kp, (int)Balance_Kd,
            (int)Velocity_Kp, (int)Velocity_Ki, Middle_angle, Voltage,
            Bluetooth_Command,
            (int)Angle_Balance, (int)Gyro_Balance,
            (unsigned long)Distance, Ultrasonic_Valid, Ultrasonic_Obstacle,
            Ultrasonic_Avoid_Enable, Ultrasonic_Avoid_Action,
            Ultrasonic_Miss_Count,
            Ultrasonic_Stop_Distance, Ultrasonic_Slow_Distance,
            Ultrasonic_Request_Speed, Ultrasonic_Safe_Speed,
            Encoder_Left, Encoder_Right,
            Motor_Left, Motor_Right);
    ascii_puts(buf);
}

static void ascii_cmd_handle(char *cmd)
{
    long a, b, c, d;

    /*
     * BAL：只改角度环。
     * 用在调参前半段：速度环保持关闭，观察 Kp/Kd 对“扶正力度”和“抖动”的影响。
     */
    if (sscanf(cmd, "BAL %ld %ld", &a, &b) == 2)
    {
        Balance_Kp = (float)a;
        Balance_Kd = (float)b;
        ascii_puts("OK,BAL\r\n");
    }
    /*
     * VEL：只改速度环。
     * 用在角度环已经能抗倒之后，观察小车是否会一直朝某个方向跑。
     */
    else if (sscanf(cmd, "VEL %ld %ld", &a, &b) == 2)
    {
        Velocity_Kp = (float)a;
        Velocity_Ki = (float)b;
        ascii_puts("OK,VEL\r\n");
    }
    /*
     * PID：一次写入四个参数。
     * 快捷键 F3-F10 使用这个命令，保证每组演示参数能快速复现。
     */
    else if (sscanf(cmd, "PID %ld %ld %ld %ld", &a, &b, &c, &d) == 4)
    {
        Balance_Kp = (float)a;
        Balance_Kd = (float)b;
        Velocity_Kp = (float)c;
        Velocity_Ki = (float)d;
        ascii_puts("OK,PID\r\n");
    }
    /* MID：调整机械直立零点。 */
    else if (sscanf(cmd, "MID %ld", &a) == 1)
    {
        Middle_angle = (int)a;
        ascii_puts("OK,MID\r\n");
    }
    /* 姿态解算方式切换：DMP / Kalman / Complementary。 */
    else if (strcmp(cmd, "MODE DMP") == 0)
    {
        Way_Angle = 1;
        ascii_puts("OK,MODE,DMP\r\n");
    }
    else if (strcmp(cmd, "MODE KALMAN") == 0)
    {
        Way_Angle = 2;
        ascii_puts("OK,MODE,KALMAN\r\n");
    }
    else if (strcmp(cmd, "MODE COMP") == 0)
    {
        Way_Angle = 3;
        ascii_puts("OK,MODE,COMP\r\n");
    }
    else if (strcmp(cmd, "ARM") == 0)
    {
        /* ARM 解除由 STOP 设置的软件停机，硬件 KEY2 仍参与安全判断。 */
        Pick_up_stop = 0;
        Flag_Stop = 0;
        Ultrasonic_Guard_SetEnable(1);
        ascii_puts("OK,ARM,KEY2_ENABLE_STILL_REQUIRED\r\n");
    }
    else if (strcmp(cmd, "UAUTO") == 0 || strcmp(cmd, "UAVOID") == 0 ||
             strcmp(cmd, "UGUARD ON") == 0)
    {
        Flag_front = 0;
        Flag_back = 0;
        Flag_Left = 0;
        Flag_Right = 0;
        Ultrasonic_Guard_SetEnable(1);
        Bluetooth_Command = BT_CMD_ULTRASONIC_AVOID;
        ascii_puts("OK,ULTRASONIC,GUARD_ON\r\n");
    }
    else if (strcmp(cmd, "UNORMAL") == 0 || strcmp(cmd, "UOFF") == 0 ||
             strcmp(cmd, "UGUARD OFF") == 0)
    {
        if (Flag_Stop == 1 || KEY2_STATE == 1)
        {
            Ultrasonic_Guard_SetEnable(0);
            Bluetooth_Motion_Stop();
            ascii_puts("OK,ULTRASONIC,GUARD_OFF_WHILE_STOPPED\r\n");
        }
        else
        {
            ascii_puts("ERR,UGUARD,STOP_REQUIRED\r\n");
        }
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        /* STOP 立即关闭四路 PWM，演示或异常时优先使用。 */
        Flag_Stop = 1;
        Bluetooth_Motion_Stop();
        PWMA_IN1 = 0;
        PWMA_IN2 = 0;
        PWMB_IN1 = 0;
        PWMB_IN2 = 0;
        ascii_puts("OK,STOP\r\n");
    }
    else if (strcmp(cmd, "STATUS") == 0)
    {
        ascii_print_status();
    }
    else if (strcmp(cmd, "HELP") == 0)
    {
        ascii_puts("CMD: BAL kp kd | VEL kp ki | PID bk bd vk vi | MID angle | MODE DMP/KALMAN/COMP | UGUARD ON/OFF | ARM | STOP | STATUS\r\n");
    }
    else
    {
        ascii_puts("ERR,UNKNOWN_CMD\r\n");
    }
}
