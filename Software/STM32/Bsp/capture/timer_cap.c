#include "timer_cap.h"

uint8_t TIM1CH1_CAPTURE_STA = 0;			    				
uint32_t TIM1CH1_CAPTURE_VAL = 0;

/**
  * @brief  
  * @param  
  * @param 
  */	
void TIM1_CH1_Cap_Init(uint32_t arr,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef  TIM1_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	   
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; 
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=arr;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);

	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 
	TIM1_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);
	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);	
	TIM_Cmd(TIM1,ENABLE ); 	

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
}

/**
  * @brief  
  * @param  
  * @param 
  */	
void TIM1_CC_IRQHandler(void)
{ 		    
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{	     
		if(TIM1CH1_CAPTURE_STA&0X40)
		{
			if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)
			{
				TIM1CH1_CAPTURE_STA|=0X80;	
				TIM1CH1_CAPTURE_VAL=0XFFFF;
			}else TIM1CH1_CAPTURE_STA++;
		}	 
	}
	if(TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
	{	
		if(TIM1CH1_CAPTURE_STA&0X40)			
		{	 
			TIM1CH1_CAPTURE_STA=0; 			
			//TIM1CH1_CAPTURE_STA|=0X80;	
			TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); 
		}
		else  	
		{
			TIM1CH1_CAPTURE_STA|=0X40;
			TIM_Cmd(TIM1,DISABLE ); 
			TIM_SetCounter(TIM1,0);
			TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);	
			TIM_Cmd(TIM1,ENABLE ); 	
		}		    
	}			     	    					   
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1|TIM_IT_Update); 
}

/**
  * @brief  
  * @param  
  * @param 
  */	
uint32_t TIM1_CapGetValue()
{
	return TIM1CH1_CAPTURE_VAL;
}