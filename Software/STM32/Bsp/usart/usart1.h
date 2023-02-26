#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stdint.h" 

void USART1_Init(uint32_t bound);
uint8_t Usart1GetChVal(uint16_t *data);

#endif


