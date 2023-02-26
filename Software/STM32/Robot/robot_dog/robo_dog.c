#include "adc.h"
#include "delay.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "usart1.h"
#include "usart3.h"
#include "timer.h"
#include "kalman.h"
#include "robot_dog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define 	RobotDogCTRL_TASK_PRIO 		2  
#define 	RobotDogCTRL_STK_SIZE 		128 

TaskHandle_t RobotDogCtrlTask_Handler; 

void RobotDogCtrlTask(void *p_arg); 
void RobotDogGPIO_Init();
void RobotDogParaInit();

void RobotDogServo_FL1(float num);
void RobotDogServo_FL2(float num);
void RobotDogServo_FL3(float num);

void RobotDogServo_FR1(float num);
void RobotDogServo_FR2(float num);
void RobotDogServo_FR3(float num);

void RobotDogServo_BL1(float num);
void RobotDogServo_BL2(float num);
void RobotDogServo_BL3(float num);

void RobotDogServo_BR1(float num);
void RobotDogServo_BR2(float num);
void RobotDogServo_BR3(float num);

Robot g_robot_dog;
DogAngle *g_dog_angle = NULL;
uint8_t g_servo_en = 0;
xSemaphoreHandle g_xRobotDogCtrlSemaphore;


/**
  * @brief  
  * @param   
  * @return 
  */
void RobotDogInit()
{
	RobotDogGPIO_Init();
	RobotDogParaInit();
#if 1
	TIM2_PWM_Init(20000-1,84-1);
	TIM3_PWM_Init(20000-1,84-1);
	TIM4_PWM_Init(20000-1,84-1);
	RobotDogServoDisable();
#endif

	xTaskCreate((TaskFunction_t )RobotDogCtrlTask,
		(const char* )"RobotDogCtrlTask",
		(uint16_t )RobotDogCTRL_STK_SIZE,
		(void* )NULL,
		(UBaseType_t )RobotDogCTRL_TASK_PRIO,
		(TaskHandle_t* )&RobotDogCtrlTask_Handler);

	vSemaphoreCreateBinary(g_xRobotDogCtrlSemaphore);
	if(g_xRobotDogCtrlSemaphore != NULL)
	{
		xSemaphoreTake(g_xRobotDogCtrlSemaphore, 0);
	}
	else
	{
		for(;;){}
	}
}

/**
  * @brief  IO初始化
  * @param   
  * @return 
  */
void RobotDogGPIO_Init()
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
}

/**
  * @brief  
  * @param  
  * @param  
  */
void RobotDogParaInit()
{
	/*电机初始位置偏差校准*/
	g_robot_dog.motor_angle_bais.bais_leg1.bais_motor1 = 10;
	g_robot_dog.motor_angle_bais.bais_leg1.bais_motor2 = 14;
	g_robot_dog.motor_angle_bais.bais_leg1.bais_motor3 = 6;

	g_robot_dog.motor_angle_bais.bais_leg2.bais_motor1 = 13;
	g_robot_dog.motor_angle_bais.bais_leg2.bais_motor2 = 15;
	g_robot_dog.motor_angle_bais.bais_leg2.bais_motor3 = 15;

	g_robot_dog.motor_angle_bais.bais_leg3.bais_motor1 = 7;
	g_robot_dog.motor_angle_bais.bais_leg3.bais_motor2 = 5;
	g_robot_dog.motor_angle_bais.bais_leg3.bais_motor3 = 10;

	g_robot_dog.motor_angle_bais.bais_leg4.bais_motor1 = -3;
	g_robot_dog.motor_angle_bais.bais_leg4.bais_motor2 = 12;
	g_robot_dog.motor_angle_bais.bais_leg4.bais_motor3 = 11;

	g_robot_dog.legFR.first_flg = 1;
	g_robot_dog.legBR.first_flg = 1;
	g_robot_dog.legBL.first_flg = 1;
	g_robot_dog.legFL.first_flg = 1;
}

/**
  * @brief 
  * @param   
  * @return 
  */
void RobotDogCtrlSemaphoreGive()
{
	portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(g_xRobotDogCtrlSemaphore, &xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken)
	{
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
  * @brief 
  * @param   
  * @return 
  */
void RobotDogCtrlTask(void *pvParameters)
{
	while(1)
	{
		if(xSemaphoreTake(g_xRobotDogCtrlSemaphore, 100000) == pdTRUE)
		{
			RobotDogCtrl();
		}
	}
}

/**
  * @brief 
  * @param   
  * @return 
  */
void RobotDogGetCtrlAngle(DogAngle *angle)
{
	g_dog_angle = angle;
}

/**
  * @brief  
  * @param  
  * @param  
  */
uint8_t RobotDogCtrlProtected(Leg *leg)
{
	if(leg->first_flg)
	{
		leg->first_flg = 0;
	}
	else
	{
		if(fabs(leg->motorValLast.hipAbducent - leg->motorVal.hipAbducent) > 20)
		{
			BEEP = 1;
			LED_RED = 1;
			return 1;
		}
		if(fabs(leg->motorValLast.hip - leg->motorVal.hip) > 20)
		{
			BEEP = 1;
			LED_RED = 1;
			return 1;
		}
		if(fabs(leg->motorValLast.knee - leg->motorVal.knee) > 20)
		{
			BEEP = 1;
			LED_RED = 1;
			return 1;
		}		
	}
	leg->motorValLast = leg->motorVal;
	return 0;
}

/**
  * @brief  
  * @param  
  * @param  
  */

void RobotDogCtrl()
{
	float angle_pre = 0.0, delta_angle = 0.0;
	static float angle_last = 0.0;

	g_robot_dog.legFR.motorVal.hipAbducent = g_dog_angle->dog_angle[0];
	g_robot_dog.legFR.motorVal.hip = g_dog_angle->dog_angle[1];
	g_robot_dog.legFR.motorVal.knee = g_dog_angle->dog_angle[2];

	g_robot_dog.legBR.motorVal.hipAbducent = g_dog_angle->dog_angle[3];
	g_robot_dog.legBR.motorVal.hip = g_dog_angle->dog_angle[4];
	g_robot_dog.legBR.motorVal.knee = g_dog_angle->dog_angle[5];

	g_robot_dog.legBL.motorVal.hipAbducent = g_dog_angle->dog_angle[6];
	g_robot_dog.legBL.motorVal.hip = g_dog_angle->dog_angle[7];
	g_robot_dog.legBL.motorVal.knee = g_dog_angle->dog_angle[8];

	g_robot_dog.legFL.motorVal.hipAbducent = g_dog_angle->dog_angle[9];
	g_robot_dog.legFL.motorVal.hip = g_dog_angle->dog_angle[10];
	g_robot_dog.legFL.motorVal.knee = g_dog_angle->dog_angle[11];

	angle_pre = g_robot_dog.legFR.motorVal.knee;
	delta_angle = fabs(angle_pre - angle_last);
	if(delta_angle > 8.0)
	{
		BEEP = 1;
	}
	else
	{
		BEEP = 0;
	}
	angle_last = angle_pre;
	if(RobotDogCtrlProtected(&g_robot_dog.legFR))
	{
		return;
	}
	if(RobotDogCtrlProtected(&g_robot_dog.legBR))
	{
		return;
	}
	if(RobotDogCtrlProtected(&g_robot_dog.legBL))
	{
		return;
	}
	if(RobotDogCtrlProtected(&g_robot_dog.legFL))
	{
		return;
	}	
	
	/*一条腿都为零，也可以作为异常姿态判断依据*/
	if((fabs(g_robot_dog.legFR.motorVal.hipAbducent) < 0.01) && (fabs(g_robot_dog.legFR.motorVal.hip) < 0.01) && (fabs(g_robot_dog.legFR.motorVal.knee) < 0.01))
	{
		return;
	}
#if 1
	RobotDogServo_FR1(g_robot_dog.legFR.motorVal.hipAbducent);
	RobotDogServo_FR2(g_robot_dog.legFR.motorVal.hip);
	RobotDogServo_FR3(g_robot_dog.legFR.motorVal.knee);

	RobotDogServo_BR1(g_robot_dog.legBR.motorVal.hipAbducent);
	RobotDogServo_BR2(g_robot_dog.legBR.motorVal.hip);
	RobotDogServo_BR3(g_robot_dog.legBR.motorVal.knee);

	RobotDogServo_BL1(g_robot_dog.legBL.motorVal.hipAbducent);
	RobotDogServo_BL2(g_robot_dog.legBL.motorVal.hip);
	RobotDogServo_BL3(g_robot_dog.legBL.motorVal.knee);	

	RobotDogServo_FL1(g_robot_dog.legFL.motorVal.hipAbducent);
	RobotDogServo_FL2(g_robot_dog.legFL.motorVal.hip);
	RobotDogServo_FL3(g_robot_dog.legFL.motorVal.knee);
#endif
}

/**
  * @brief  
  * @param  
  * @param  
  */
void RobotDogServoDisable()
{
	TIM_SetCompare2(TIM4,0);
	TIM_SetCompare3(TIM4,0);
	TIM_SetCompare4(TIM4,0);
	TIM_SetCompare1(TIM2,0);
	TIM_SetCompare2(TIM2,0);
	TIM_SetCompare3(TIM2,0);
	TIM_SetCompare3(TIM3,0);
	TIM_SetCompare4(TIM3,0);
	TIM_SetCompare1(TIM4,0);
	TIM_SetCompare4(TIM2,0);
	TIM_SetCompare1(TIM3,0);
	TIM_SetCompare2(TIM3,0);
}

/**
  * @brief  
  * @param  
  * @param  
  */
void RobotDogServoEnCtrl(uint8_t is_en)
{
	g_servo_en = is_en;
	g_robot_dog.en = g_servo_en;  //添加判断，奇异角度不使能
	if(!is_en)
	{
		RobotDogServoDisable();
	}
}

/**
  * @brief 
  * @param   
  * @return 
  */
uint8_t RobotDogGetServoEnStatus()
{
	return g_robot_dog.en;
}

void RobotDogServo_FL1(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>270)
	{
		return;
	} 
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg4.bais_motor1) * 2000.0 / CORRECT_270; 
	if(g_servo_en)  //定义为带参数宏
		TIM_SetCompare2(TIM4,angle);
	else
		TIM_SetCompare2(TIM4,0);
}

void RobotDogServo_FL2(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg4.bais_motor2) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare3(TIM4,angle);
	else
		TIM_SetCompare3(TIM4,0);
}

void RobotDogServo_FL3(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg4.bais_motor3) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare4(TIM4,angle);
	else
		TIM_SetCompare4(TIM4,0);
}

void RobotDogServo_FR1(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>270)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg1.bais_motor1) * 2000.0 / CORRECT_270;  //
	if(g_servo_en)
		TIM_SetCompare1(TIM2,angle);
	else
		TIM_SetCompare1(TIM2,0);
}

void RobotDogServo_FR2(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg1.bais_motor2) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare2(TIM2,angle);
	else
		TIM_SetCompare2(TIM2,0);
}

void RobotDogServo_FR3(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg1.bais_motor3) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare3(TIM2,angle);
	else
		TIM_SetCompare3(TIM2,0);
}

void RobotDogServo_BL1(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>270)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg3.bais_motor1) * 2000.0 / CORRECT_270;
	if(g_servo_en)
		TIM_SetCompare3(TIM3,angle);
	else
		TIM_SetCompare3(TIM3,0);
}

void RobotDogServo_BL2(float num)
{
	uint16_t angle = 0;
	
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg3.bais_motor2) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare4(TIM3,angle);
	else
		TIM_SetCompare4(TIM3,0);
}

void RobotDogServo_BL3(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg3.bais_motor3) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare1(TIM4,angle);
	else
		TIM_SetCompare1(TIM4,0);
}

void RobotDogServo_BR1(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>270)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg2.bais_motor1) * 2000.0 / CORRECT_270;
	if(g_servo_en)
		TIM_SetCompare4(TIM2,angle);
	else
		TIM_SetCompare4(TIM2,0);
}

void RobotDogServo_BR2(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg2.bais_motor2) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare1(TIM3,angle);
	else
		TIM_SetCompare1(TIM3,0);
}

void RobotDogServo_BR3(float num)
{
	uint16_t angle = 0;
	if(num<0 || num>180)
	{
		return;
	}
	angle = 500.0 + (num + g_robot_dog.motor_angle_bais.bais_leg2.bais_motor3) * 2000.0 / CORRECT_180;
	if(g_servo_en)
		TIM_SetCompare2(TIM3,angle);
	else
		TIM_SetCompare2(TIM3,0);
}









