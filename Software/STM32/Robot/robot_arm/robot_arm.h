#ifndef _ARM_H
#define _ARM_H
#include "sys.h"

#define     ABS(a)       (a > 0 ? (a) : (-a))
#define     SIGN(a)       (a > 0 ? (1) : (-1))

#define     ANGLE_CONVERT_RATE          100.0
#define     ARM_CTRL_SAFE_ANGLE         10.0

#define  	FRAME_HEAD_1			0xFA
#define  	FRAME_HEAD_2			0xAF
#define  	FRAME_TAIL				0xED
#define  	CMD_POS_CTRL			0x01
#define  	CMD_POS_READ			0x02
#define  	CMD_RESET_ID			0xCD

#define EN_T            PDout(0)	
#define EN_R            PDout(1)
#define SERVO_SMALL     0
#define SERVO_BIG       1

typedef struct
{
    float axis_angle[7];  //7个电机
}ArmAngle;

typedef struct
{
    uint16_t pos;
    uint8_t id;
    uint8_t read_flg;
    float offset;
    float last_ctrl_angle;
}ServoInfo;

void RobotArmInit();
void RobotArmServoFeedback(char data[10]);
void RobotArmAngleRes();
void RobotArmAngleSemaphoreGive();
void RobotArmCtrlTask(void *pvParameters);
void RobotArmCtrlSemaphoreGive();
void RobotArmServoStartPosRead();
void RobotArmAngleCtrlValue(ArmAngle *angle);
void RobotArmAllServoPosCtrl();
int RobotArmAxis2AngleConvert(float in);
int RobotArmServoAngleOptimizing(float angle);
void RobotArmServoPosCtrl(uint8_t id, uint8_t goal_angle, uint8_t run_time, uint16_t lock_time);
#endif


