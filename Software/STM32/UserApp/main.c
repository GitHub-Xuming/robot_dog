#include "robot_dog.h"
#include "robot_arm.h"
#include "timer.h"
#include "remote_ctrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SEGGER_RTT.h"

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	
	delay_init(168);
	SysTimerInit(1000 - 1, 84 - 1);
	SeggerInit();

	RemoteCtrlInit();
	ComHostInit();
	RobotArmInit();
	RobotDogInit();

	vTaskStartScheduler();
}





