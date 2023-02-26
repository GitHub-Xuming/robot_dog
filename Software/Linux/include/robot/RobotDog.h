#ifndef __ROBOT_DOG_H__
#define __ROBOT_DOG_H__

#include <stdint.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <memory>
#include <thread>
#include "DataProgress.h"
#include "filter.h"
#include "struct.h"
#include "DogCtrlBase.h"
#include "MatrixCalc.hpp"



/********-- body尺寸相关 --********/

#define ROT_Y             (PI / 2.0)
#define REDUCE_RATE       2.3939 

#define LEG_2_SWING_STSRT      0/16.0
#define LEG_2_SWING_END        4/16.0

#define LEG_1_SWING_STSRT      4/16.0
#define LEG_1_SWING_END        8/16.0

#define LEG_3_SWING_STSRT      8/16.0
#define LEG_3_SWING_END        12/16.0

#define LEG_4_SWING_STSRT      12/16.0
#define LEG_4_SWING_END        15/16.0

#define    GAIT_S    160
#define    GAIT_H    40

typedef enum
{ 
	leg_1 = 0,
	leg_2,
	leg_3,
	leg_4
}RobotLeg; 

typedef struct
{ 
	RobotLeg leg_number;
	Gait_para gait;
	Pose special_point;  
	Pose world_coor_point; 
	Pose local_coor_point; 
	Joint primitiveVal; 
	Joint fiveLinkVal;
	Joint motorVal; 
	Pose bodyPeakPoint; 
}Leg;

typedef struct
{ 
	float rot_axis_x;
	float rot_axis_y;
	float rot_axis_z;
}BodyRot;

typedef struct
{ 
	float move_axis_x;
	float move_axis_y;
	float move_axis_z;
}BodyMove;

typedef struct
{ 
	float move_x;  //用于body相对于foot的移动
	float move_y;
	float move_z;
	BodyRot rotateVal;
	BodyMove moveVal;
	Pose bodyCoorZeroPoint;
	float body_centre_move_x;
	float body_centre_move_y;
	float body_centre_move_z;
	float postureMatrix[4][4];
	int move_vel; //身体移动速度

}Body;

typedef struct
{
	Body body;

	Leg legFR;
	Leg legBR;
	Leg legBL;
	Leg legFL;

}Robot_;

class RobotDog : public DogCtrlBase, public MatrixCalc
{
public:
    RobotDog();
    ~RobotDog();
	void RobotDogPoseFeedback(Pose &pose);
	void GetAngleAndPost(float angle[4][3],float pos[2][3]);
	void RobotDogServoEnCtrl(bool ctrl);
	void ServoAngleConstruct();

	virtual void RobotDogCtrl(Pose posture, FootCoord foot_pos, std::vector<uint8_t> &out) override;
	virtual void EN_Ctrl(bool en) override;

    Robot_ robot;
	Pose m_robot_pose_;
	std::thread m_robot_thread_;
	bool m_thread_run_;
	bool m_robot_enable_ = false;

private:
    void RobotDogConvertToMotorAngle(const float in_angle[3],float out_angle[3],RobotLeg leg );
    void RobotDogAngleConvert(const float in_angle[3],float out_angle[3],RobotLeg leg);
    void LegInverseResolving(Leg *leg_kind);
    void RobotDogInvForward(Leg *leg_kind);
    void RobotDogRotMove(BodyRot rotateVal,BodyMove moveVal);
	void RobotDogProcessFlow();
	void RobotDogRun();
	void GetFiveLinkAngle(Leg *leg_kind);
	void GetFiveLinkToMotor(Leg *leg_kind);

private:
	std::vector<uint8_t> m_send_data_;
	std::shared_ptr<BlockQueue<std::vector<char>>> m_send_queue_;
};


#endif 

