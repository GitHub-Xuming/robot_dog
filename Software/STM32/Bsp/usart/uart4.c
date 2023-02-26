#include "sys.h"
#include "uart4.h"
#include "FreeRTOS.h"
#include "robot_arm.h"

char g_arm_buffer[10] = { 0 };

/**
  * @brief  
  * @param  
  * @param 
  */
void Uart4Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
  
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
    USART_Init(UART4, &USART_InitStructure); 
    USART_Cmd(UART4, ENABLE); 
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);	
}

/**
  * @brief  
  * @param  
  */
void Uart4SendByte(uint8_t data)
{
	USART_SendData(UART4, data);
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET){} 
}

/**
  * @brief  
  * @param  
  * @param 
  */
void UART4_IRQHandler(void)   
{
	volatile uint8_t temp = 0;
	char pre_receive_data = 0;
	uint8_t sum = 0;
	static int counter = 0;
	static char last_receive_data = 0;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) 
	{
		pre_receive_data = USART_ReceiveData(UART4); 
		
		if( (counter == 0) && ((last_receive_data != 0xFA) || (pre_receive_data != 0xAF)) )
		{
			g_arm_buffer[0] = pre_receive_data; 
			last_receive_data = pre_receive_data;
			return;      //第 0 号数据不是帧头，跳过
		}
		
		counter++; 
		g_arm_buffer[counter] = pre_receive_data; 
		
		if(counter == 10) 
		{ 
			counter = 0; 
			for(int i = 2; i < 8; i++)
			{
				sum += g_arm_buffer[i];
			}
			if((sum == g_arm_buffer[8]) && (g_arm_buffer[9] == 0xED))
			{
				RobotArmServoFeedback(g_arm_buffer);
			}
		}
	}
	USART_ClearFlag(UART4, USART_FLAG_RXNE);
	temp = UART4->SR;
	temp = UART4->DR;
} 



