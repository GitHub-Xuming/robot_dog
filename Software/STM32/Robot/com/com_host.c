#include "sys.h"
#include "usart3.h"
#include "com_host.h"

#define     COM_TASK_PRIO           4 
#define     COM_STK_SIZE            256 
#define     COM_HOST_BAUD_RATE      1000000 
#define     CMD_QUEUE_NUM           10

TaskHandle_t COM_Task_Handler; 
void ComHostSendTask(void *p_arg); 
static xQueueHandle com_send_queue = NULL; 

/**
  * @brief  
  * @param  
  * @param 
  */
void ComHostInit()
{
    USART3_Init(COM_HOST_BAUD_RATE);

    if(NULL == com_send_queue)
    {
        com_send_queue = xQueueCreate(CMD_QUEUE_NUM, sizeof(ACK_FRAM));
    }

	xTaskCreate((TaskFunction_t )ComHostSendTask,
		(const char* )"ComHostSendTask",
		(uint16_t )COM_STK_SIZE,
		(void* )NULL,
		(UBaseType_t )COM_TASK_PRIO,
		(TaskHandle_t* )&COM_Task_Handler);
}

/**
  * @brief  
  * @param  
  * @param 
  */
void ComHostSend(ACK_FRAM *data)
{
    uint8_t sum = 0;

    if(NULL == data)
    {
        return;
    }
	Uart3SendData(UART_DATA_HEAD); 
	Uart3SendData(data->function_code); sum += data->function_code;
	Uart3SendData(data->data_len);  sum += data->data_len;
	for(int i = 0; i < data->data_len; i++)
	{
		Uart3SendData(data->data[i]); 
        sum += data->data[i];
	}
	Uart3SendData(sum);
}

/**
  * @brief  
  * @param  
  * @param 
  */
BaseType_t ComHostPacketToQueue(ACK_FRAM *pack, uint8_t is_from_ISR)
{
    BaseType_t send_res = pdFALSE;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(NULL == com_send_queue)
    {
        return pdFALSE;
    }
    if(is_from_ISR)
    {
        send_res = xQueueSendFromISR(com_send_queue, pack, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else
    {
        send_res = xQueueSend(com_send_queue, pack, 10000);
    }
    return send_res;
}

/**
  * @brief  
  * @param  
  * @param 
  */
void ComHostSendTask(void *pvParameters)
{
    static ACK_FRAM package;

    while(1)
    {
        if(xQueueReceive(com_send_queue, &package, 100000) == pdTRUE)
        {
            //send data
            ComHostSend(&package);
        }
    }
}





