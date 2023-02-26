#ifndef __TIMER_CAP_H
#define __TIMER_CAP_H
#include "sys.h"
#include "stdint.h"

void TIM1_CH1_Cap_Init(uint32_t arr,uint16_t psc);
uint32_t TIM1_CapGetValue();

#endif























