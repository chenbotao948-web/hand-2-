/* usart.h */
#ifndef __USART_H
#define __USART_H

#include "main.h"

void UART_Init(void);
void UART_Task(void);
void UART_SendString(const char *str);
void UART_IR(void);

#endif
