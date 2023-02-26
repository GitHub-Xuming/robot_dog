#ifndef __UART_4_H
#define __UART_4_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stdint.h" 

void Uart4Init(uint32_t bound);
void Uart4SendByte(uint8_t data);

#endif


