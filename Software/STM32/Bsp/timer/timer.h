#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "stdint.h"

#define SYS_CLOCK_TIMER                 	TIM2
#define RCC_APB1Periph_SYS_CLOCK_TIMER   	RCC_APB1Periph_TIM2
#define SYS_CLOCK_TIMER_IRQn   			 	TIM2_IRQn
#define SYS_CLOCK_TIMER_IRQHandler 	    	TIM2_IRQHandler

void SysTimerInit(uint16_t arr,uint16_t psc);
uint32_t SysTimerGetTimes();

#endif


