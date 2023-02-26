#ifndef __ROBOT_H
#define __ROBOT_H

#include <math.h>
#include <stdint.h> 
#include <sys.h>
#include "pwm.h"

#define PI                3.141592654
#define CORRECT_180       198.77
#define CORRECT_270       290.43
#define REDUCE_RATE       2.3939  

#define BEEP 			PDout(10)
#define LED_RED 		PDout(11)
#define LED_GREEN 		PDout(12)



typedef struct
{
    float dog_angle[12];
}DogAngle;

typedef struct
{
    int bais_motor1;
    int bais_motor2;
    int bais_motor3;
}bais_motor;

typedef struct
{
    bais_motor bais_leg1;
    bais_motor bais_leg2;
    bais_motor bais_leg3;
    bais_motor bais_leg4;
}adj_motor;

typedef enum
{ 
	leg_1 = 0,
	leg_2,
	leg_3,
	leg_4
}RobotLeg; 


typedef struct
{ 
	float hipAbducent; 
	float hip; 
	float knee; 
}Joint;  

typedef struct
{ 
	RobotLeg leg_number;
	Joint motorVal; 
	Joint motorValLast;
	uint8_t first_flg;
}Leg;

typedef struct
{
	Leg legFR;
	Leg legBR;
	Leg legBL;
	Leg legFL;
	adj_motor motor_angle_bais;
	uint8_t en;
}Robot;

void RobotDogInit();
void RobotDogCtrlSemaphoreGive();
void RobotDogCtrlTask(void *pvParameters);
void RobotDogCtrl();
void RobotDogGetCtrlAngle(DogAngle *angle);

void RobotDogServoDisable();
uint8_t RobotDogGetServoEnStatus();
void RobotDogServoEnCtrl(uint8_t is_en);


#endif 



   