
#ifndef __STRUCT_H__
#define __STRUCT_H__

#include <stdint.h>
#include <vector>
#include <functional>
#include "sem.h"
#include "CubicInterpolation.h"

/*运动相关参数*/
#define 	INTERPOLATION_DURATION                	3
#define 	INTERPOLATION_PERIOD                	0.02
/*转换相关*/
#define 	PI                			3.141592654
#define 	DEGRE2RAD(angle)			(angle * PI / 180.0)
#define 	RAD2DEGRE(angle)			(angle * 180.0 / PI)
#define 	STDARD_ANGLE(angle)			((angle) < 0 ? (angle + 360.0) : (angle))
#define     ANGLE_CONVERT_RATE          100.0
/*串口相关*/
#define     UART_DATA_HEAD         		0x55  //串口通信帧头
#define     RCV_TYPE_REMOTE_CTRL        0x51  //遥控数据

#define     TYPE_ARM_FIRST_ANGLE_REQ    0x42  //机械臂初始角度请求功能码
#define     TYPE_IMU_POS_REQ            0x43  //IMU位置请求

#define     TYPE_IMU_POS_RES            0x56  //IMU位置回复
#define     TYPE_ARM_POS_RES        	0x52  //机械臂位置回复
#define     TYPE_ARM_POS_CTR            0x53  //机械臂位置控制
#define     TYPE_DOG_POS_CTR            0x54  //RobotDog位置控制


#define     RECEIVE_MAX_NUM         	30
#define     HEART_BEAT              	0x61

/*RobotDog Body尺寸*/
#define DOG_BODY_LENGTH       341.0
#define DOG_BODY_WIDE         133.0  

/*RobotDog 连杆尺寸*/
#define DOG_L1                45.75
#define DOG_L2                82.0
#define DOG_L3                226.0 
#define DOG_MID_L3            96.0
#define DOG_LEFT_L1           42.3
#define DOG_LEFT_L2           137.0
#define DOG_A                 45.0
#define DOG_A2                DOG_L2
#define DOG_A3                16.0
#define DOG_D3                DOG_L1
#define DOG_D4                DOG_L3

/*RobotDog 身体初始位置*/
#define FIRST_BODY_POS_X  350
#define FIRST_BODY_POS_Y  375
#define FIRST_BODY_POS_Z  260 

/*RobotDog 初始位置定义*/
#define BAIS_X                (-DOG_A/2.0-20)
#define ADJ_X                 0  
#define ADJ_Y                 20  
#define FOOT_1_X              (FIRST_BODY_POS_X - DOG_BODY_LENGTH / 2.0 + BAIS_X - ADJ_X)
#define FOOT_1_Y              (FIRST_BODY_POS_Y + DOG_BODY_WIDE / 2.0 + DOG_L1 - ADJ_Y)
#define FOOT_1_Z              0.0
#define FOOT_2_X              (FIRST_BODY_POS_X + DOG_BODY_LENGTH / 2.0 + BAIS_X + ADJ_X)
#define FOOT_2_Y              (FIRST_BODY_POS_Y + DOG_BODY_WIDE / 2.0 + DOG_L1 - ADJ_Y)
#define FOOT_2_Z              0.0
#define FOOT_3_X              (FIRST_BODY_POS_X + DOG_BODY_LENGTH / 2.0 + BAIS_X + ADJ_X)
#define FOOT_3_Y              (FIRST_BODY_POS_Y - DOG_BODY_WIDE / 2.0 - DOG_L1 + ADJ_Y)
#define FOOT_3_Z              0.0
#define FOOT_4_X              (FIRST_BODY_POS_X - DOG_BODY_LENGTH / 2.0 + BAIS_X - ADJ_X)
#define FOOT_4_Y              (FIRST_BODY_POS_Y - DOG_BODY_WIDE / 2.0 - DOG_L1 + ADJ_Y)
#define FOOT_4_Z              0.0

/*RobotDog 足端初始位置*/
#define GAIT_FIRST_POINT_1_X        FOOT_1_X
#define GAIT_FIRST_POINT_1_Y        FOOT_1_Y
#define GAIT_FIRST_POINT_1_Z        FOOT_1_Z

#define GAIT_FIRST_POINT_2_X        FOOT_2_X
#define GAIT_FIRST_POINT_2_Y        FOOT_2_Y
#define GAIT_FIRST_POINT_2_Z        FOOT_2_Z

#define GAIT_FIRST_POINT_3_X        FOOT_3_X 
#define GAIT_FIRST_POINT_3_Y        FOOT_3_Y
#define GAIT_FIRST_POINT_3_Z        FOOT_3_Z

#define GAIT_FIRST_POINT_4_X        FOOT_4_X
#define GAIT_FIRST_POINT_4_Y        FOOT_4_Y
#define GAIT_FIRST_POINT_4_Z        FOOT_4_Z

enum ArmRunMode
{
	AxisMode,
	CoordMode
};

typedef struct
{ 
	float hipAbducent; 
	float hip; 
	float knee; 
}Joint; 

typedef struct
{ 
	float x;
	float y;
	float z;
	float pitch;
	float roll;
	float yaw;
}Pose; 

typedef struct
{
	float x;
	float y;
	float z;
}coord_point;

typedef struct
{ 
	float axis_1;
	float axis_2;
	float axis_3;
	float axis_4;
	float axis_5;
	float axis_6;
}AxisAngle;

typedef struct
{
	uint8_t head;
    uint8_t function_code;
    uint8_t data_len;
    uint8_t check_sum;
    std::vector<uint8_t> data;

}SERIAL_SEND;

typedef struct
{
	coord_point foot_FR;
    coord_point foot_FL;
    coord_point foot_BR;
    coord_point foot_BL;
}FootCoord;


typedef struct
{
	float claw_angle;
    AxisAngle axis_angle;
}ArmAngleFeedBack;

typedef struct
{
	CubicPtr interpolater;
	float angle;
    float para_b;
	float para_k;
}ArmAxis;

typedef struct
{
	Joint leg_FR;
    Joint leg_FL;
	Joint leg_BR;
    Joint leg_BL;
}RobotDogJointPos;

enum RobotOperationStatus
{
    ArmReset,
    ArmStretch,
	ArmInCtrl,
    ArmContract,
	DogStretch,
	Disable,
};

/*数据请求模型*/
struct DataReqModel
{
    uint8_t func_code;
	uint8_t data_len;
	uint8_t retry_times;
	uint32_t time_out_ms;
    std::vector<uint8_t> data;
	ToolSem sem;
};

using RobotRemotFeedBack = std::vector<uint16_t>;
using ArmAngleCallback = std::function<void (ArmAngleFeedBack &)>;
using IMU_PosCallback = std::function<void (coord_point &)>;

using RobotDogRemotCallback = std::function<void (RobotRemotFeedBack &)>;
using RobotDogPoseFeedbackCallback = std::function<void (Pose &)>;
using VirtualExcuterGetPosCallback = std::function<float (void)>;

#endif
