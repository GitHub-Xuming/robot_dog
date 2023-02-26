#ifndef COM_HOST_H__
#define COM_HOST_H__
#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"


typedef struct
{
	uint8_t head;
    uint8_t function_code;
    uint8_t data_len;
    uint8_t data[30];
}ACK_FRAM;


void ComHostInit();
BaseType_t ComHostPacketToQueue(ACK_FRAM *pack, uint8_t is_from_ISR);
void ComHostSend(ACK_FRAM *data);


#endif