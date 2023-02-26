#ifndef __UART_5_H
#define __UART_5_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stdint.h" 

#define     FRAME_HEAD      0xFC
#define     FRAME_END       0xFD
#define     TYPE_IMU        0x40
#define     TYPE_AHRS       0x41

#define     TYPE_GROUND     0xF0

#define 	TYPE_INSGPS 	0x42
#define 	TYPE_VEL 		0x60
#define 	TYPE_ACC 		0x62
#define 	TYPE_ANGLE 		0x63

#define 	ACC_LEN 		16 
#define 	VEL_LEN 		12 
#define 	ANGLE_LEN 		12 
#define 	INSGPS_LEN 		72

#define     IMU_LEN         0x38   //56+8  8组数据
#define     AHRS_LEN        0x30   //48+8  7组数据

#define     IMU_CAN         9
#define     AHRS_CAN        8
#define     INSGPS_CAN      11

void USART5_Init(uint32_t bound);
#endif


