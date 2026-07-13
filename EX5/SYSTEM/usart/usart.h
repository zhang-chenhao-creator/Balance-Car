#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

void uart_init(u32 bound);
void usart1_send(u8 data);

#endif
