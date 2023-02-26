#include "timer.h"
#include "math.h"

/*系统时间*/
volatile uint32_t g_sys_time = 0;

/**
  * @brief  通用定时器中断初始化
  * @param  arr：自动重装值
  * @param  psc：时钟预分频数
  * @return none
  */
void SysTimerInit(uint16_t arr, uint16_t psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SYS_CLOCK_TIMER,ENABLE); 
	
    TIM_TimeBaseInitStructure.TIM_Period = arr;
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; 
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(SYS_CLOCK_TIMER,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(SYS_CLOCK_TIMER,TIM_IT_Update,ENABLE);
	TIM_Cmd(SYS_CLOCK_TIMER,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel=SYS_CLOCK_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  get全局系统运行时间
  * @param  
  * @param 
  */
uint32_t SysTimerGetTimes()
{
	return g_sys_time;
}

/**
  * @brief  
  * @param  
  * @param 
  */
void SYS_CLOCK_TIMER_IRQHandler(void)
{
	if(TIM_GetITStatus(SYS_CLOCK_TIMER,TIM_IT_Update)==SET) 
	{
		g_sys_time ++;
	}
	TIM_ClearITPendingBit(SYS_CLOCK_TIMER,TIM_IT_Update); 
}
