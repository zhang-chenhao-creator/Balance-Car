#ifndef __USART3_H
#define __USART3_H

#include "stm32f10x.h"

#define BT_CMD_STOP       0
#define BT_CMD_FORWARD    1
#define BT_CMD_BACKWARD   2
#define BT_CMD_LEFT       3
#define BT_CMD_RIGHT      4
#define BT_CMD_ULTRASONIC_AVOID 5

void uart3_init(u32 bound);
void Bluetooth_Command_Process(u8 command);
void Bluetooth_Motion_Stop(void);
void USART3_IRQHandler(void);

extern volatile u8 Bluetooth_Command;

#endif
