#include "sys.h"
#include "robot_arm.h"
#include "usart3.h"
#include "robot_dog.h"
#include "FreeRTOS.h"
#include "timer.h"
#include "inertial_nav.h"

#define USART3_RX_BUFFER_SIZE 100

int g_sum_false = 0;
ArmAngle g_u3_arm_angle;
DogAngle g_u3_dog_angle;
unsigned char g_usart3_decode_buffer[64] = { 0 };
uint8_t g_usart3_recv_buffer[USART3_RX_BUFFER_SIZE] = { 0 };

/**
  * @brief  
  * @param DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
  * @param chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
  * @param par:外设地址
  * @param mar:存储器地址
  * @param ndtr:数据传输量 
  * @return none
  */ 
static void Usart3DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,uint32_t chx,uint32_t par,uint32_t mar,uint32_t dir,uint32_t ndtr)
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
static void Usart3DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx, uint32_t ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                      //先关闭DMA,才能设置它
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//等待传输结束	
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //设置传输数据长度 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA
}	  

/**
  * @brief  串口3初始化
  * @param  bound:波特率
  * @return 
  */
void USART3_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3,&USART_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启接收中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启空闲中断
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//开启DMA接收
	USART_Cmd(USART3, ENABLE);
	
	//initialize the DMA channel.
	Usart3DMA_Config(DMA1_Stream1, DMA_Channel_4, 
						 (uint32_t)&(USART3->DR),     //串口DR寄存器
						 (uint32_t)g_usart3_recv_buffer,  //自定义的接收数据buf
						 DMA_DIR_PeripheralToMemory,  //外设到存储器方向
						 USART3_RX_BUFFER_SIZE);      //长度
}

/**
  * @brief  数据解码
  * @param  data:传入数据指针地址
  * @param  len:数据长度
  * @return 
  */
float test_A_1, test_A_2, test_A_3, test_A_4, test_A_5, test_A_6;
static void Usart3DataDecode(uint8_t *data, uint16_t len)
{
	Position tmp_pos;
	uint8_t sum = 0;
    if(len > 32)
    {
        g_sum_false ++;
        return;
    }
    memcpy(g_usart3_decode_buffer, data, len); //缓存数据

    for(int i = 1; i < len - 1; i++)  //校验 功能码到data
    {
        sum += g_usart3_decode_buffer[i]; //进行校验
    }  

    if(g_usart3_decode_buffer[len - 1] == sum && g_usart3_decode_buffer[0] == UART_DATA_HEAD)
    {
        switch(g_usart3_decode_buffer[1])
        {
            case SEND_TYPE_ARM_POS_REQ:  
            //RobotArmAngleRes();
            RobotArmAngleSemaphoreGive();
            break;

            case TYPE_IMU_POS_REQ:  
			IntertialPosRes();
            break;

            case TYPE_ARM_POS_CTR:  
            for(int i = 0; i < 7; i++)
            {
                g_u3_arm_angle.axis_angle[i] = ((g_usart3_decode_buffer[i * 2 + 3] << 8) | (g_usart3_decode_buffer[i * 2 + 4])) / ANGLE_CONVERT_RATE;
            }
			/*机械臂末端位置，上位机乘以10下发*/
			tmp_pos.x = ((int16_t)((g_usart3_decode_buffer[14 + 3] << 8) | (g_usart3_decode_buffer[15 + 3]))) / 10.0;
			tmp_pos.y = ((int16_t)((g_usart3_decode_buffer[16 + 3] << 8) | (g_usart3_decode_buffer[17 + 3]))) / 10.0;
			tmp_pos.z = ((int16_t)((g_usart3_decode_buffer[18 + 3] << 8) | (g_usart3_decode_buffer[19 + 3]))) / 10.0;
			NaviTailPos(tmp_pos);

            RobotArmAngleCtrlValue(&g_u3_arm_angle);
            test_A_1 = g_u3_arm_angle.axis_angle[0];
            test_A_2 = g_u3_arm_angle.axis_angle[1];
            test_A_3 = g_u3_arm_angle.axis_angle[2];
            test_A_4 = g_u3_arm_angle.axis_angle[3];
            test_A_5 = g_u3_arm_angle.axis_angle[4];
            test_A_6 = g_u3_arm_angle.axis_angle[5];
            RobotArmCtrlSemaphoreGive();
            break;

            case TYPE_DOG_POS_CTR:  
            for(int i = 0; i < 12; i++)
            {
                g_u3_dog_angle.dog_angle[i] = ((g_usart3_decode_buffer[i * 2 + 3] << 8) | (g_usart3_decode_buffer[i * 2 + 4])) / ANGLE_CONVERT_RATE;
            }
            RobotDogGetCtrlAngle(&g_u3_dog_angle);
            RobotDogCtrlSemaphoreGive();
            break;

            default:  break;
        }	
    }
    else
    {
        g_sum_false ++;
    }
}

/**
  * @brief  
  * @param   
  * @return 
  */
void USART3_IRQHandler(void) 
{
    volatile uint8_t temp = 0;
	uint8_t rc_tmp = 0;
	uint16_t rc_len = 0;

	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		rc_tmp = USART3->SR;
		rc_tmp = USART3->DR;//软件序列清除IDLE标志位
		DMA_Cmd(DMA1_Stream1, DISABLE);//关闭DMA，准备重新配置
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);	// Clear Transfer Complete flag
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TEIF1);	// Clear Transfer error flag	
		rc_len = USART3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);//计算接收数据长度
		
		Usart3DataDecode(g_usart3_recv_buffer, rc_len);//解码收到的数据
	}
	Usart3DMA_Enable(DMA1_Stream1, USART3_RX_BUFFER_SIZE);//开启下一次DMA接收
}

/**
  * @brief  
  * @param   
  * @return 
  */
void Uart3SendData(uint8_t data)
{
	USART_SendData(USART3, data);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET){} 
}

