#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__

#include <stdint.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <memory>
#include <thread>
#include "DataProgress.h"
#include "ArmCtrlBase.h"
#include "struct.h"
#include "MatrixCalc.hpp"


namespace arm
{

#define 	ARM_START_POINT_X		(78.05 - 150.0)
#define 	ARM_START_POINT_Y		(375.0 - 0.0)
#define 	ARM_START_POINT_Z		(307.35 + 100.0)

/********-- ARM尺寸 --********/

#define ARM_L1                0
#define ARM_L2                230.0
#define ARM_L3                230.8

#define ARM_A2                ARM_L2
#define ARM_A3                33.0
#define ARM_D3                ARM_L1
#define ARM_D4                ARM_L3

typedef struct
{
	float x;
	float z;
}coord_xz;
 
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

class RobotArm : public ArmCtrlBase, public MatrixCalc
{
public:
    RobotArm();
    ~RobotArm();

	void RegistPoseFeedbackImpl(RobotDogPoseFeedbackCallback callback);
    void RobotArmConvert(BodyRot rotateVal,BodyMove moveVal);
	void RobotArmMatrixPrint(const float m[4][4], std::string matix_name);
    void RobotArmRun();
	void InverseResolving();
    virtual void ArmCoordinateMode(Pose point, AxisAngle &result) override;
    virtual void ArmAxisMode(AxisAngle angle) override;
    virtual void ClawCtrl(float angle) override;

    float m_angle_1_;
    float m_angle_2_;
    float m_angle_3_;
    float m_angle_4_;
    float m_angle_5_;
    float m_angle_6_;

private:
	std::thread m_thread_;
	bool m_thread_run_;
	bool m_robot_enable_;

	Pose world_coor_point; //世界坐标系描述的点
	float local_coor_point[4][4]; //局部坐标系描述的点
	float bodyPostureMatrix[4][4];

	BodyRot rotateVal;
	BodyMove moveVal;
	ArmRunMode m_run_mode_; 
	RobotDogPoseFeedbackCallback m_pose_callback_ = nullptr;
	Pose m_robot_dog_pose_;
};

}
#endif 

