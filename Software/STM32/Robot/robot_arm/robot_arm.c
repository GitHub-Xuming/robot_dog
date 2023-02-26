#include "robot_arm.h"
#include "math.h"
#include "usart3.h"
#include "uart4.h"
#include "delay.h"
#include "SEGGER_RTT.h"
#include "com_host.h"
#include "remote_ctrl.h"
#include "inertial_nav.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define 	RobotArmCTRL_TASK_PRIO 			2  
#define 	RobotArmCTRL_STK_SIZE 			128 
#define 	ARM_ANGLE_ASK_TASK_PRIO 		2  
#define 	ARM_ANGLE_ASK_STK_SIZE 			128 
#define     ARM_CTRL_BAUD_RATE      		1000000 
#define     IMU_COM_BAUD_RATE      			921600 

TaskHandle_t Arm_Angle_Ask_Task_Handler; 
TaskHandle_t RobotArmCtrlTask_Handler;
void RobotArmAngleAskTask(void *p_arg); 
void RobotArmCtrlTask(void *p_arg); 
void RobotArmGPIO_Init();

uint8_t g_send_buff[10] = {FRAME_HEAD_1, FRAME_HEAD_2, 0x04, 0xD4, 0x00, 0x00, 0x00, 0x00};
uint8_t g_servo_kind = 0;
uint8_t g_id_convert_tb[7] = {1, 21, 3, 4, 5, 6, 7};

ACK_FRAM g_arm_res;
ServoInfo g_servo_info[7] = { 0 };
ArmAngle *g_arm_angle = NULL;
xSemaphoreHandle g_xArmAngleResSemaphore;
xSemaphoreHandle g_xRobotArmCtrlSemaphore;

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmInit()
{
	Adc_Init();
	RobotArmGPIO_Init();
	Uart4Init(ARM_CTRL_BAUD_RATE); //机械臂
	//USART5_Init(IMU_COM_BAUD_RATE); //imu
	TIM1_CH1_Cap_Init(0xFFFF, 840 - 1); //光流传感器
	NavigationInit();
	/*servo 零点偏置初始化 原则：将舵机位置摆放到机械臂坐标系零点位置后，读取的角度值即为:offset*/
	g_servo_info[0].offset = 0;
	g_servo_info[1].offset = 0;
	g_servo_info[2].offset = 0;
	g_servo_info[3].offset = 0;
	g_servo_info[4].offset = 0;
	g_servo_info[5].offset = 120;
	g_servo_info[6].offset = 0;

	/*舵机id赋值，位置和读取标志清零*/
	for(int i = 0; i < 7; i++)
	{
		g_servo_info[i].id = g_id_convert_tb[i];
		g_servo_info[i].pos = 0;
		g_servo_info[i].read_flg = 0;
	}
	xTaskCreate((TaskFunction_t )RobotArmAngleAskTask,
		(const char* )"RobotArmAngleAskTask",
		(uint16_t )ARM_ANGLE_ASK_STK_SIZE,
		(void* )NULL,
		(UBaseType_t )ARM_ANGLE_ASK_TASK_PRIO,
		(TaskHandle_t* )&Arm_Angle_Ask_Task_Handler);

	xTaskCreate((TaskFunction_t )RobotArmCtrlTask,
		(const char* )"RobotArmCtrlTask",
		(uint16_t )RobotArmCTRL_STK_SIZE,
		(void* )NULL,
		(UBaseType_t )RobotArmCTRL_TASK_PRIO,
		(TaskHandle_t* )&RobotArmCtrlTask_Handler);
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmGPIO_Init()
{ 
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_SetBits(GPIOD,GPIO_Pin_0 | GPIO_Pin_1);
	/*Arm角度请求信号量创建*/
	vSemaphoreCreateBinary(g_xArmAngleResSemaphore);
	if(g_xArmAngleResSemaphore != NULL)
	{
		xSemaphoreTake(g_xArmAngleResSemaphore, 0);
	}
	else
	{
		for(;;){}
	}
	/*Arm控制信号量创建*/
	vSemaphoreCreateBinary(g_xRobotArmCtrlSemaphore);
	if(g_xRobotArmCtrlSemaphore != NULL)
	{
		xSemaphoreTake(g_xRobotArmCtrlSemaphore, 0);
	}
	else
	{
		for(;;){}
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmAngleSemaphoreGive()
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(g_xArmAngleResSemaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmCtrlSemaphoreGive()
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(g_xRobotArmCtrlSemaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmAngleRes()
{
	g_arm_res.head = UART_DATA_HEAD;
	g_arm_res.function_code = TYPE_ARM_POS_RES; 
	g_arm_res.data_len = 14; 
	uint16_t tmp = 0;
	for(int i = 0; i < 7; i++)
	{
		tmp = g_servo_info[i].pos * ANGLE_CONVERT_RATE;
		g_arm_res.data[i * 2] = ((tmp >> 8) & 0x00ff);
		g_arm_res.data[i * 2 + 1] = (tmp & 0x00ff);
	}
	ComHostPacketToQueue(&g_arm_res, 0);
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmAngleAskTask(void *p_arg)
{
	int flag_sum = 0;
	while(1)
	{
		if(xSemaphoreTake(g_xArmAngleResSemaphore, 100000) == pdTRUE)
		{
			USART_Cmd(UART4, ENABLE); 
			while(1)
			{
				RobotArmServoStartPosRead();  //正常情况下，发送函数执行完成，已经收到舵机回复了
				for(int i = 0; i < 7; i++)
				{
					flag_sum += g_servo_info[i].read_flg;
				}
				if(7 == flag_sum)
				{
					break;
				}
				flag_sum = 0;
				vTaskDelay(20);
			}
			USART_Cmd(UART4, DISABLE);  
			for(int i = 0; i < 7; i++)
			{
				g_servo_info[i].read_flg = 0;
			}
			RobotArmAngleRes();
		}
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmAngleCtrlValue(ArmAngle *angle)
{
	g_arm_angle = angle;
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmCtrlTask(void *pvParameters)
{
	float delta_angle = 0.0;
	while(1)
	{
		if(xSemaphoreTake(g_xRobotArmCtrlSemaphore, 100000) == pdTRUE)
		{
			//send angle to servo
			RobotArmAllServoPosCtrl();
		}
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmAllServoPosCtrl()
{
	int ctrl_angle = 0;
	float delta_angle = 0.0;

	Uart4Init(115200);
	g_servo_kind = SERVO_BIG;
	for(int i = 0; i < 3; i++)
	{
		ctrl_angle = RobotArmServoAngleOptimizing(g_arm_angle->axis_angle[i]);
		delta_angle = ctrl_angle - g_servo_info[i].last_ctrl_angle; //突变计算
		if(ABS(delta_angle) < ARM_CTRL_SAFE_ANGLE)  //位置突变保护
		{
			if(i == 1)
			{
				RobotArmServoPosCtrl(22, RobotArmAxis2AngleConvert(ctrl_angle), 0, 0);
				vTaskDelay(2);  //必要延时，舵机回复数据较慢
			}
			RobotArmServoPosCtrl(g_id_convert_tb[i], ctrl_angle, 0, 0);
			vTaskDelay(2);
			g_servo_info[i].last_ctrl_angle = ctrl_angle;
		}
	}

	Uart4Init(1000000);
	g_servo_kind = SERVO_SMALL;
	for(int i = 3; i < 7; i++)
	{
		/*对控制的float角度进行优化处理*/
		ctrl_angle = RobotArmServoAngleOptimizing(g_arm_angle->axis_angle[i]);
		delta_angle = ctrl_angle - g_servo_info[i].last_ctrl_angle; //突变计算
		if(ABS(delta_angle) < ARM_CTRL_SAFE_ANGLE)  //位置突变保护
		{
			RobotArmServoPosCtrl(g_id_convert_tb[i], ctrl_angle, 0, 0);
			vTaskDelay(1);
			g_servo_info[i].last_ctrl_angle = ctrl_angle;
		}
	}
}

// float ConvertServo2Arm(ServoInfo *servo, float angle)
// {
// 	float out = 0;
// 	out = angle - servo->offset;
// 	return out < 0 ? (out + 360) : out;
// }

// float ConvertArm2Servo(ServoInfo *servo, float angle)
// {
// 	float out = 0;
// 	out = angle + servo->offset;
// 	return out > 360 ? (out - 360) : out;
// }

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmCkeckAndSend()
{
	uint8_t check_sum = 0;

	for(int i = 2; i < 8; i++)
	{
		check_sum += g_send_buff[i];
	}
	g_send_buff[8] = check_sum;
	g_send_buff[9] = FRAME_TAIL;
	
	EN_T = 0;
	EN_R = 1;
	for(int i = 0; i < 10; i++)
	{
		Uart4SendByte(g_send_buff[i]);
	}
	if(g_servo_kind == SERVO_SMALL)
	{
		delay_us(30);  //min 6us
	}
	if(g_servo_kind == SERVO_BIG)
	{
		delay_us(70);  //min 6us
	}
	EN_T = 1;
	EN_R = 0;
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmServoPosCtrl(uint8_t id, uint8_t goal_angle, uint8_t run_time, uint16_t lock_time)
{
	g_send_buff[2] = id;
	g_send_buff[3] = CMD_POS_CTRL;
	g_send_buff[4] = goal_angle;
	g_send_buff[5] = run_time;
	g_send_buff[6] = lock_time >> 8;
	g_send_buff[7] = lock_time & 0xFF;
	RobotArmCkeckAndSend();
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmServoPosRead(uint8_t id)
{
	g_send_buff[2] = id;
	g_send_buff[3] = CMD_POS_READ;
	g_send_buff[4] = 0;
	g_send_buff[5] = 0;
	g_send_buff[6] = 0;
	g_send_buff[7] = 0;
	RobotArmCkeckAndSend();
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmServoIdReset(uint8_t id, uint8_t new_id)
{
	g_send_buff[2] = id;
	g_send_buff[3] = CMD_RESET_ID;
	g_send_buff[4] = 0;
	g_send_buff[5] = new_id;
	g_send_buff[6] = 0;
	g_send_buff[7] = 0;
	RobotArmCkeckAndSend();
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmServoFeedback(char data[10])
{
	char servo_id = 0, servo_angle = 0;

	servo_id = data[2];
	servo_angle = data[7];

	for(int i = 0; i < 7; i++)
	{
		if(g_servo_info[i].id == servo_id)
		{
			g_servo_info[i].pos = servo_angle;
			g_servo_info[i].last_ctrl_angle = (float)servo_angle; //把初始读角度值作为上次角度，以用于位置突变计算
			g_servo_info[i].read_flg = 1;
		}
	}
}

/**
  * @brief  
  * @param  
  * @param 
  */
int RobotArmServoAngleOptimizing(float angle)
{
	int out = 0, tmp = 0;

	tmp = (int)angle;
	if((angle - tmp) > 0.5)
	{
		out = tmp + 1;
	}
	else
	{
		out = tmp;
	}
	return out;
}

/**
  * @brief  
  * @param  
  * @param 
  */
int RobotArmAxis2AngleConvert(float in)
{
	float angle = 0.0;
	int out = 0;
	
	angle = -0.9948 * in + 175.18;
	out = RobotArmServoAngleOptimizing(angle);
	return out;
}

/**
  * @brief  
  * @param  
  * @param 
  */
void RobotArmServoStartPosRead()
{
	Uart4Init(1000000);
	g_servo_kind = SERVO_SMALL;
	for(int i = 3; i < 7; i++)
	{
		RobotArmServoPosRead(g_id_convert_tb[i]);
		vTaskDelay(2);
	}
	Uart4Init(115200);
	g_servo_kind = SERVO_BIG;
	RobotArmServoPosRead(22);  //22号舵机先读一次，读取可以Disable电机
	vTaskDelay(2);
	for(int i = 0; i < 3; i++)
	{
		RobotArmServoPosRead(g_id_convert_tb[i]);
		vTaskDelay(2);
	}
	
	//test_out = servoAngleConvert(test_in);
	//SEGGER_RTT_printf(0, "%d  %d\n", g_servo_info[1], g_servo_info[2]);
}
