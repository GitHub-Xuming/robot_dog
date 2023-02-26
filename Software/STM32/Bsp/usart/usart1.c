#include "sys.h"
#include "usart1.h"
#include "FreeRTOS.h"
#include "stm32f4xx_dma.h"

#define USART1_RX_BUFFER_SIZE 35

unsigned char g_usart5_decode_buffer[32] = { 0 };
uint16_t g_channels[10] = { 0 };
uint32_t g_ibus_receive_error_count = 0;
uint8_t g_usart5_recv_buffer[USART1_RX_BUFFER_SIZE];

/**
  * @brief  
  * @param DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
  * @param chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
  * @param par:外设地址
  * @param mar:存储器地址
  * @param ndtr:数据传输量 
  * @return none
  */ 
void Usart1DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,uint32_t chx,uint32_t par,uint32_t mar,uint32_t dir,uint32_t ndtr)
{ 
	DMA_InitTypeDef  DMA_InitStructure;

	if((uint32_t)DMA_Streamx>(uint32_t)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	}else 
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_DeInit(DMA_Streamx);

	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 

	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel 					= chx;  							//通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr 		= par;								//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr 			= mar;								//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR 					    = dir;								//direction of transmit.
	DMA_InitStructure.DMA_BufferSize 				= ndtr;								//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc				= DMA_PeripheralInc_Disable;		//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc 				= DMA_MemoryInc_Enable;				//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;		//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;			//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode 						= DMA_Mode_Normal;					// 使用普通模式 
	DMA_InitStructure.DMA_Priority 					= DMA_Priority_High;				//中等优先级
	DMA_InitStructure.DMA_FIFOMode 					= DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;			//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst 		    = DMA_PeripheralBurst_Single;		//外设突发单次传输
	DMA_Init(DMA_Streamx, &DMA_InitStructure);
	DMA_Cmd(DMA_Streamx,ENABLE);
} 

/**
  * @brief  开启一次DMA传输
  * @param   
  * @return 
  */
void Usart1DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,uint32_t ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                      //先关闭DMA,才能设置它
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//等待传输结束	
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //设置传输数据长度 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA
}	  

/**
  * @brief  串口初始化
  * @param  bound:波特率
  * @return 
  */
void USART1_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_Initstructure;
	USART_InitTypeDef USART_Initstructure;
	NVIC_InitTypeDef NVIC_Initstrcuture;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA ,ENABLE );
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);
	
	USART_Initstructure.USART_BaudRate = bound;
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Initstructure.USART_Parity = USART_Parity_No;
	USART_Initstructure.USART_StopBits = USART_StopBits_1;
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_Initstructure);
	
	NVIC_Initstrcuture.NVIC_IRQChannel = USART1_IRQn;
	NVIC_Initstrcuture.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_Initstrcuture.NVIC_IRQChannelSubPriority =2;		
	NVIC_Initstrcuture.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_Initstrcuture);	
	
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);//开启DMA接收
	USART_Cmd(USART1, ENABLE);
	
	//initialize the DMA channel.
	Usart1DMA_Config(DMA2_Stream2,DMA_Channel_4, 
						 (uint32_t)&(USART1->DR),     //串口DR寄存器
						 (uint32_t)g_usart5_recv_buffer,//自定义的接收数据buf
						 DMA_DIR_PeripheralToMemory,//外设到存储器方向
						 USART1_RX_BUFFER_SIZE);//长度
}

/**
  * @brief  数据解码
  * @param  data:传入数据指针地址
  * @param  len:数据长度
  * @return 
  */
void Usart1Data_Decode(uint8_t *data, uint16_t len)
{
	uint16_t check_sum = 0, sum = 0;
	if(len != 32)
	{
		g_ibus_receive_error_count ++;
		return;
	}
	memcpy(g_usart5_decode_buffer, data, len); //缓存数据
	for(int i = 0; i < len - 2; i++) //从帧头开始校验
	{
		check_sum += g_usart5_decode_buffer[i];
	}
	check_sum = ~check_sum; //取反
	sum = *((uint16_t*)(&data[30]));
	if(check_sum != sum)
	{
		g_ibus_receive_error_count ++;
		return;
	}
	for(int i = 0; i < 10; i++)
	{
		g_channels[i] = *((uint16_t*)(&data[2 + i * 2]));
	}
}

/**
  * @brief  
  * @param   
  * @return 
  */
void USART1_IRQHandler(void) 
{
	uint8_t rc_tmp;
	uint16_t rc_len;

	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)
	{
		rc_tmp=USART1->SR;
		rc_tmp=USART1->DR;//软件序列清除IDLE标志位
		DMA_Cmd(DMA2_Stream2, DISABLE);//关闭DMA，准备重新配置
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);	// Clear Transfer Complete flag
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);	// Clear Transfer error flag	
		rc_len = USART1_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2);//计算接收数据长度
		
		Usart1Data_Decode(g_usart5_recv_buffer, rc_len);//解码收到的数据
	}
	Usart1DMA_Enable(DMA2_Stream2,USART1_RX_BUFFER_SIZE);//开启下一次DMA接收
}

/**
  * @brief  获取遥控器通道数据
  * @param  data:传出数据地址
  * @return 
  */
uint8_t Usart1GetChVal(uint16_t *data)
{
	uint32_t ch_sum = 0;

	for(int i = 0; i < 10; i++)
	{
		ch_sum += g_channels[i];
	}
	if(ch_sum < 9000)  //每个通道从1000到2000，最小总和10000，这里取90000
	{
		return 0; //遥控器没打开时，通道值为0，防止误操作
	}
	else
	{
		for(int i = 0; i < 10; i++)
		{
			data[i] = g_channels[i] - 1000;
		}
		return 1;
	}
}
