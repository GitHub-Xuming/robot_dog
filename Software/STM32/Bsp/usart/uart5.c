#include "sys.h"
#include "uart5.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "robot_dog.h"
#include "inertial_nav.h"

#define UART5_RX_BUFFER_SIZE 64

IMUData_Packet_t IMUData_Packet;
uint8_t g_imu_decode_buffer[UART5_RX_BUFFER_SIZE];
uint8_t g_imu_recv_buffer[UART5_RX_BUFFER_SIZE];

/**
  * @brief  
  * @param DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
  * @param chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
  * @param par:外设地址
  * @param mar:存储器地址
  * @param ndtr:数据传输量 
  * @return none
  */ 
static void Uart5DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,uint32_t chx,uint32_t par,uint32_t mar,uint32_t dir,uint32_t ndtr)
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
	DMA_Cmd(DMA_Streamx, ENABLE);
} 

/**
  * @brief  开启一次DMA传输
  * @param   
  * @return 
  */
static void Uart5DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, uint32_t ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                      //先关闭DMA,才能设置它
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//等待传输结束	
	DMA_SetCurrDataCounter(DMA_Streamx, ndtr);          //设置传输数据长度 
	DMA_Cmd(DMA_Streamx, ENABLE);                       //开启DMA
}	  

/**
  * @brief  串口初始化
  * @param  bound:波特率
  * @return 
  */
void USART5_Init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_Initstructure;
	USART_InitTypeDef USART_Initstructure;
	NVIC_InitTypeDef NVIC_Initstrcuture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE); 

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); 
	
    GPIO_Initstructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_Initstructure); 

	GPIO_Initstructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_OType = GPIO_OType_OD; 
	GPIO_Initstructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_Initstructure); 
	
	USART_Initstructure.USART_BaudRate = bound;
	USART_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Initstructure.USART_Parity = USART_Parity_No;
	USART_Initstructure.USART_StopBits = USART_StopBits_1;
	USART_Initstructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(UART5,&USART_Initstructure);
	
	NVIC_Initstrcuture.NVIC_IRQChannel = UART5_IRQn;
	NVIC_Initstrcuture.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Initstrcuture.NVIC_IRQChannelSubPriority = 2;		
	NVIC_Initstrcuture.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_Initstrcuture);	
	
//	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(UART5, ENABLE);
	
	//initialize the DMA channel.
	Uart5DMA_Config(DMA1_Stream0, DMA_Channel_4, 
						 (uint32_t)&(UART5->DR),    //串口DR寄存器
						 (uint32_t)g_imu_recv_buffer,//自定义的接收数据buf
						 DMA_DIR_PeripheralToMemory,//外设到存储器方向
						 UART5_RX_BUFFER_SIZE);//长度
}

/**
  * @brief  数据解码
  * @param  data:传入数据指针地址
  * @param  len:数据长度
  * @return 
  */
static void Uart5Data_Decode(uint8_t *data, uint16_t len)
{
	memcpy(g_imu_decode_buffer, data, len); //缓存数据

	if((g_imu_decode_buffer[len - 1] == FRAME_END) && (g_imu_decode_buffer[0] == FRAME_HEAD))
	{
		switch(g_imu_decode_buffer[1])
		{
			case TYPE_ACC: //标识这个包是加速度包
			memcpy(&IMUData_Packet.acc_x, &g_imu_decode_buffer[7], 12);
			break;

			case TYPE_ANGLE: //标识这个包是角度包
			memcpy(&IMUData_Packet.Roll, &g_imu_decode_buffer[7], 12);
			InertialNav(IMUData_Packet); //角度包在后，在此一并传递加速度数据
			break;

			// case TYPE_VEL:
			// memcpy(&IMUData_Packet.vel_x, &g_imu_decode_buffer[7], 12);
			// break;

			// case TYPE_INSGPS:
			// memcpy(&IMUData_Packet.BodyVelocity_X, &g_imu_decode_buffer[7], 12);
			// memcpy(&IMUData_Packet.Location_North, &g_imu_decode_buffer[7 + 24], 12);

			default:  
			break;
		}	
	}
}

/**
  * @brief  
  * @param   
  * @return 
  */
void UART5_IRQHandler(void) 
{
	volatile uint8_t temp = 0;
	uint8_t rc_tmp = 0;
	uint16_t rc_len = 0;

	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
	{
		rc_tmp=UART5->SR;
		rc_tmp=UART5->DR;//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream0, DISABLE);//关闭DMA，准备重新配置
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);	// Clear Transfer Complete flag
		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TEIF0);	// Clear Transfer error flag	
		rc_len = UART5_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream0);//计算接收数据长度
		
		Uart5Data_Decode(g_imu_recv_buffer, rc_len);//解码收到的数据
	}
	Uart5DMA_Enable(DMA1_Stream0, UART5_RX_BUFFER_SIZE);//开启下一次DMA接收
	temp = UART5->SR;
	temp = UART5->DR;
}

