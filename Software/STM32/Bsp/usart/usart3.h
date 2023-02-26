#ifndef __USART_3_H
#define __USART_3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stdint.h"


/*串口相关*/
#define     UART_DATA_HEAD         		0x55
#define     RCV_TYPE_REMOTE_CTRL        0x51
#define     SEND_TYPE_ARM_POS_REQ       0x42  //机械臂位置请求功能码
#define     TYPE_IMU_POS_REQ            0x43  //IMU位置请求


#define     TYPE_IMU_POS_RES            0x56  //IMU位置回复
#define     TYPE_ARM_POS_RES            0x52  //机械臂位置回复
#define     TYPE_ARM_POS_CTR            0x53  //机械臂位置控制
#define     TYPE_DOG_POS_CTR            0x54 

#define     RECEIVE_MAX_NUM         	30
#define     HEART_BEAT              	0x61

void USART3_Init(uint32_t bound);
void Uart3SendData(uint8_t data);

#endif


