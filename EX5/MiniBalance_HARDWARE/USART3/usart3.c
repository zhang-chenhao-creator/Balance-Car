#include "sys.h"

/*
 * USART3 Bluetooth control
 *
 * Hardware:
 *   PB10 = USART3_TX
 *   PB11 = USART3_RX
 *
 * Role in this project:
 *   The Bluetooth module sends one-byte commands. This file converts those
 *   commands into motion flags used by the speed loop and turning loop.
 *
 * Control flow:
 *   phone/controller -> USART3_IRQHandler -> Bluetooth_Command_Process
 *   -> Flag_front/Flag_back/Flag_Left/Flag_Right
 *   -> control.c Velocity()/Turn()
 */
#define BT_SPEED_MIN 100
#define BT_SPEED_MAX 600
#define BT_SPEED_STEP 100

static void Bluetooth_Clear_Motion(void)
{
    /* Only one motion direction is active at a time. */
    Flag_front = 0;
    Flag_back = 0;
    Flag_Left = 0;
    Flag_Right = 0;
}

void Bluetooth_Motion_Stop(void)
{
    Bluetooth_Clear_Motion();
    Bluetooth_Command = BT_CMD_STOP;
}

static void Bluetooth_Set_Motion(u8 state)
{
    Bluetooth_Clear_Motion();
    Bluetooth_Command = state;

    /*
     * Forward/backward affect the velocity target.
     * Left/right affect the turn target.
     */
    if (state == BT_CMD_FORWARD)
        Flag_front = 1;
    else if (state == BT_CMD_BACKWARD)
        Flag_back = 1;
    else if (state == BT_CMD_LEFT)
        Flag_Left = 1;
    else if (state == BT_CMD_RIGHT)
        Flag_Right = 1;
}

void uart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* PB10: TX, alternate-function push-pull. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* PB11: RX, floating input. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void Bluetooth_Command_Process(u8 command)
{
    /*
     * The remote sends ASCII letters on some Bluetooth apps and small numeric
     * codes on others, so both formats are accepted here.
     */
    switch (command)
    {
        case 'A':
        case 0x01:
            Bluetooth_Set_Motion(BT_CMD_FORWARD);
            break;

        case 'E':
        case 0x05:
            Bluetooth_Set_Motion(BT_CMD_BACKWARD);
            break;

        case 'B':
        case 'C':
        case 'D':
        case 0x02:
        case 0x03:
        case 0x04:
            Bluetooth_Set_Motion(BT_CMD_RIGHT);
            break;

        case 'F':
        case 'G':
        case 'H':
        case 0x06:
        case 0x07:
        case 0x08:
            Bluetooth_Set_Motion(BT_CMD_LEFT);
            break;

        case 'X':
            /* Increase target speed in small steps, with saturation. */
            if (Target_Velocity <= BT_SPEED_MAX - BT_SPEED_STEP)
                Target_Velocity += BT_SPEED_STEP;
            else
                Target_Velocity = BT_SPEED_MAX;
            break;

        case 'Y':
            /* Decrease target speed but keep a useful minimum. */
            if (Target_Velocity >= BT_SPEED_MIN + BT_SPEED_STEP)
                Target_Velocity -= BT_SPEED_STEP;
            else
                Target_Velocity = BT_SPEED_MIN;
            break;

        case 'U':
        case 'u':
            /*
             * Ultrasonic autonomous avoidance mode:
             * the control loop generates forward/back/turn behavior according
             * to the measured front distance. Existing manual Bluetooth
             * commands remain available after sending STOP.
             */
            Bluetooth_Clear_Motion();
            Ultrasonic_Guard_SetEnable(1);
            Bluetooth_Command = BT_CMD_ULTRASONIC_AVOID;
            break;

        case 'Z':
        case 0x00:
            Bluetooth_Motion_Stop();
            break;

        default:
            break;
    }
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        u8 command = (u8)USART_ReceiveData(USART3);
        Bluetooth_Command_Process(command);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
