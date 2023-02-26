#include "usart1.h"	 
#include "usart3.h"	
#include "delay.h"	
#include "robot_dog.h"
#include "remote_ctrl.h"	
#include "FreeRTOS.h"
#include "task.h"
#include "com_host.h"

#define 	REMOT_CTRL_TASK_PRIO 		3 
#define 	REMOT_CTRL_STK_SIZE 		128 
#define 	REMOT_CH_NUM 				10
#define     REMOTE_BAUD_RATE      		115200 


TaskHandle_t REMOT_CTRL_Task_Handler; 
void RemoteCtrlTask(void *p_arg); 

uint16_t remot_ch_val[REMOT_CH_NUM] = { 0 };
uint8_t send_data[20] = { 0 };
ACK_FRAM remote_data;

/**
  * @brief  
  * @param  
  * @param 
  */
void RemoteCtrlInit()
{
	USART1_Init(REMOTE_BAUD_RATE); 
	xTaskCreate((TaskFunction_t )RemoteCtrlTask,
		(const char* )"RemoteCtrlTask",
		(uint16_t )REMOT_CTRL_STK_SIZE,
		(void* )NULL,
		(UBaseType_t )REMOT_CTRL_TASK_PRIO, 
		(TaskHandle_t* )&REMOT_CTRL_Task_Handler);
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RemoteCtrlDataPacket()
{
	remote_data.head = UART_DATA_HEAD;
	remote_data.function_code = RCV_TYPE_REMOTE_CTRL; 
	remote_data.data_len = 20; 
	for(int i = 0; i < 20; i++)
	{
		remote_data.data[i] = send_data[i];
	}
	ComHostPacketToQueue(&remote_data, 0);
}

/**
  * @brief  
  * @param  
  * @param  
  */
void RemoteCtrlProcess() 
{
	if(Uart4SendByte(remot_ch_val))
	{
		for(int i = 0; i < REMOT_CH_NUM; i++)
		{
			send_data[i * 2] = ((remot_ch_val[i] >> 8) & 0xff);
			send_data[i * 2 + 1] = (remot_ch_val[i] & 0xff);
		}
		RemoteCtrlDataPacket();
		if((remot_ch_val[5 - 1] > 200) && (remot_ch_val[5 - 1] < 1200))
		{
			RobotDogServoEnCtrl(1);
		}
		else
		{
			RobotDogServoEnCtrl(0);
		}
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RemoteCtrlTask(void *pvParameters)
{
	while(1)
	{
		RemoteCtrlProcess();
		vTaskDelay(100); //10Hz
	}
}

