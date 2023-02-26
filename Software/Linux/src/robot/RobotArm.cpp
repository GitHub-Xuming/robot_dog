#include <math.h>
#include "RobotArm.h"
#include "my_debug.h"
#include "DataProgress.h"

using namespace std;
using namespace arm;

RobotArm::RobotArm()
{
	world_coor_point.x = ARM_START_POINT_X;
	world_coor_point.y = ARM_START_POINT_Y;
	world_coor_point.z = ARM_START_POINT_Z;

	world_coor_point.roll = DEGRE2RAD(0);
	world_coor_point.pitch = DEGRE2RAD(0);
	world_coor_point.yaw = DEGRE2RAD(0);

	m_thread_run_ = true;
	RobotArmRun();
}

RobotArm::~RobotArm()
{
	m_thread_run_ = false;
}

void RobotArm::RegistPoseFeedbackImpl(RobotDogPoseFeedbackCallback callback)
{
	m_pose_callback_ = callback;
}

void RobotArm::ArmCoordinateMode(Pose point, AxisAngle &result)
{
	world_coor_point = point;
	m_run_mode_ = CoordMode;  

	/*控制body姿态，三个平移和旋转*/
	RobotArmConvert(rotateVal, moveVal);
	/*逆解，得到原始角度值*/
	InverseResolving();	
	result.axis_1 = m_angle_1_;
	result.axis_2 = m_angle_2_;
	result.axis_3 = m_angle_3_;
	result.axis_4 = m_angle_4_;
	result.axis_5 = m_angle_5_;
	result.axis_6 = m_angle_6_;
}

void RobotArm::ArmAxisMode(AxisAngle angle)
{
	m_angle_1_ = angle.axis_1;
	m_angle_2_ = angle.axis_2;
	m_angle_3_ = angle.axis_3;
	m_angle_4_ = angle.axis_4;
	m_angle_5_ = angle.axis_5;
	m_angle_6_ = angle.axis_6;

	m_run_mode_ = AxisMode;
}

void RobotArm::ClawCtrl(float angle)
{
}

/**
  * @brief  
  * @param  
  * @param  
  */
void RobotArm::RobotArmRun()
{
    m_thread_ = std::thread([this]
    {
        while(m_thread_run_)
		{
			//TODO:
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));    
		}
    });
}

void RobotArm::RobotArmMatrixPrint(const float m[4][4], std::string matix_name)
{
}

void RobotArm::RobotArmConvert(BodyRot rotateVal,BodyMove moveVal)
{
	float move[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_x[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_y[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_z[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_1[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_2[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_3[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_inv[3] = { 0 }; 

	float rot_test_x[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_test_y[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_test_z[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };

	if(m_pose_callback_ != nullptr)
		m_pose_callback_(m_robot_dog_pose_);
	/*矩阵初始化*/
	move[0][3] = m_robot_dog_pose_.x;
	move[1][3] = m_robot_dog_pose_.y;
	move[2][3] = m_robot_dog_pose_.z;
	
	rot_x[1][1] = cos(m_robot_dog_pose_.roll);
	rot_x[1][2] = -sin(m_robot_dog_pose_.roll);
	rot_x[2][1] = sin(m_robot_dog_pose_.roll);
	rot_x[2][2] = cos(m_robot_dog_pose_.roll);
	
	rot_y[0][0] = cos(m_robot_dog_pose_.pitch);
	rot_y[0][2] = sin(m_robot_dog_pose_.pitch);
	rot_y[2][0] = -sin(m_robot_dog_pose_.pitch);
	rot_y[2][2] = cos(m_robot_dog_pose_.pitch);	
	
	rot_z[0][0] = cos(m_robot_dog_pose_.yaw);
	rot_z[0][1] = -sin(m_robot_dog_pose_.yaw);
	rot_z[1][0] = sin(m_robot_dog_pose_.yaw);
	rot_z[1][1] = cos(m_robot_dog_pose_.yaw);
	
	MatrixCalc::MatrixMull(move,rot_x,mid_matrix_1);      			
	MatrixCalc::MatrixMull(mid_matrix_1,rot_y,mid_matrix_2);	
	MatrixCalc::MatrixMull(mid_matrix_2,rot_z,mid_matrix_1);
	move[0][3] = -271.95;
	move[1][3] = 0;
	move[2][3] = 47.35;
	MatrixCalc::MatrixMull(mid_matrix_1,move,mid_matrix_2);  //robot dog centor -> arm base
	
	rot_z[0][0] = cos(DEGRE2RAD(180));
	rot_z[0][1] = -sin(DEGRE2RAD(180));
	rot_z[1][0] = sin(DEGRE2RAD(180));
	rot_z[1][1] = cos(DEGRE2RAD(180));
	MatrixCalc::MatrixMull(mid_matrix_2,rot_z,mid_matrix_1);  //robot dog coor -> arm coor convert 
	MatrixCalc::MatrixTransmit(mid_matrix_1,bodyPostureMatrix); 

	move[0][3] = world_coor_point.x;
	move[1][3] = world_coor_point.y;
	move[2][3] = world_coor_point.z;

	rot_x[1][1] = cos(world_coor_point.roll);
	rot_x[1][2] = -sin(world_coor_point.roll);
	rot_x[2][1] = sin(world_coor_point.roll);
	rot_x[2][2] = cos(world_coor_point.roll);
	
	rot_y[0][0] = cos(world_coor_point.pitch);
	rot_y[0][2] = sin(world_coor_point.pitch);
	rot_y[2][0] = -sin(world_coor_point.pitch);
	rot_y[2][2] = cos(world_coor_point.pitch);	
	
	rot_z[0][0] = cos(world_coor_point.yaw);
	rot_z[0][1] = -sin(world_coor_point.yaw);
	rot_z[1][0] = sin(world_coor_point.yaw);
	rot_z[1][1] = cos(world_coor_point.yaw);	

	rot_test_y[0][0] = cos(DEGRE2RAD(-90));
	rot_test_y[0][2] = sin(DEGRE2RAD(-90));
	rot_test_y[2][0] = -sin(DEGRE2RAD(-90));
	rot_test_y[2][2] = cos(DEGRE2RAD(-90));	

	rot_test_z[0][0] = cos(DEGRE2RAD(180));
	rot_test_z[0][1] = -sin(DEGRE2RAD(180));
	rot_test_z[1][0] = sin(DEGRE2RAD(180));
	rot_test_z[1][1] = cos(DEGRE2RAD(180));

	MatrixCalc::MatrixMull(move,rot_test_y,mid_matrix_1);      	 
	MatrixCalc::MatrixMull(mid_matrix_1,rot_test_z,mid_matrix_2);
	MatrixCalc::MatrixMull(mid_matrix_2,rot_x,mid_matrix_1);
    
	MatrixCalc::MatrixMull(mid_matrix_1,rot_y,mid_matrix_2);
	MatrixCalc::MatrixMull(mid_matrix_2,rot_z,mid_matrix_1);

	MatrixCalc::MatrixInv(bodyPostureMatrix, mid_matrix_3);
	MatrixCalc::MatrixMull(mid_matrix_3,mid_matrix_1,mid_matrix_2);
	MatrixCalc::MatrixTransmit(mid_matrix_2,local_coor_point);

	move[0][3] = 0;
	move[1][3] = 0;
	move[2][3] = 0; //-120 
	MatrixCalc::MatrixMull(local_coor_point,move,mid_matrix_1);
	MatrixCalc::MatrixTransmit(mid_matrix_1,local_coor_point);
}

void RobotArm::InverseResolving()
{
	float theta_1 = 0,theta_2 = 0,theta_23 = 0,theta_3 = 0,K = 0;
	float theta_4 = 0,theta_5 = 0,theta_6 = 0;
	float px = 0,py = 0,pz = 0;
	float sqrt1 = 0,sqrt2 = 0;

	px = local_coor_point[0][3];
	py = local_coor_point[1][3];
	pz = local_coor_point[2][3];

	sqrt1 = (px*px + py*py - ARM_D3*ARM_D3);
	if(sqrt1 < 0)
	{
		ERROR("sqrt1 < 0!");
		return;
	}
	theta_1 = atan2(py, px) - atan2(ARM_D3, sqrt(sqrt1));
    K = (px*px + py*py + pz*pz - ARM_A2*ARM_A2 - ARM_A3*ARM_A3 - ARM_D3*ARM_D3 - ARM_D4*ARM_D4)/(2.0 * ARM_A2);
	sqrt2 = (ARM_A3*ARM_A3 + ARM_D4*ARM_D4 - K*K);
	if(sqrt2 < 0) 
	{
		ERROR("sqrt2 < 0!");
		return;
	}	    

	theta_3 = atan2(ARM_A3,ARM_D4) - atan2(K,sqrt(sqrt2));
	theta_23 = atan2((-ARM_A3 - ARM_A2 * cos(theta_3)) * pz - (cos(theta_1) * px + sin(theta_1) * py) * (ARM_D4 - ARM_A2 * sin(theta_3)), 
	(ARM_A2 * sin(theta_3) - ARM_D4) * pz + (ARM_A3 + ARM_A2 * cos(theta_3)) * (cos(theta_1) * px + sin(theta_1) * py));
    theta_2 = theta_23 - theta_3;	
	
	if(theta_1 < 0)
	{
		theta_1 = 2.0 * PI + theta_1;
	}
	if(theta_2 < 0)
	{
		theta_2 = 2.0 * PI + theta_2;
	}
	if(theta_3 < 0)
	{
		theta_3 = 2.0 * PI + theta_3;
	}

    float ms4 = -local_coor_point[0][2] * sin(theta_1) + local_coor_point[1][2] * cos(theta_1);
    float mc4 = -local_coor_point[0][2] * cos(theta_1) * cos(theta_23) - local_coor_point[1][2] * sin(theta_1) * cos(theta_23) +
	local_coor_point[2][2] * sin(theta_23);
	theta_4 = atan2(ms4, mc4);

	float ms5 = -local_coor_point[0][2] * (cos(theta_1) * cos(theta_23) * cos(theta_4) + sin(theta_1) * sin(theta_4)) -
	local_coor_point[1][2] * (sin(theta_1) * cos(theta_23) * cos(theta_4) - cos(theta_1) * sin(theta_4)) + local_coor_point[2][2] *
	(sin(theta_23) * cos(theta_4));
	
    float mc5 = local_coor_point[0][2] * (-cos(theta_1) * sin(theta_23)) + local_coor_point[1][2] * (-sin(theta_1) * sin(theta_23)) +
	local_coor_point[2][2] * (-cos(theta_23));
    theta_5 = atan2(ms5, mc5);

	float ms6 = -local_coor_point[0][0] * (cos(theta_1) * cos(theta_23) * sin(theta_4) - sin(theta_1) * cos(theta_4)) - 
	local_coor_point[1][0] * (sin(theta_1) * cos(theta_23) * sin(theta_4) + cos(theta_1) * cos(theta_4)) + 
	local_coor_point[2][0] * (sin(theta_23) * sin(theta_4));

    float mc6 = local_coor_point[0][0] * (cos(theta_1) * cos(theta_23) * cos(theta_4) + sin(theta_1) * sin(theta_4)) * cos(theta_5) -
    local_coor_point[0][0] * cos(theta_1) * sin(theta_23) * sin(theta_5) + local_coor_point[1][0] * (sin(theta_1) * cos(theta_23) *
	cos(theta_4) - cos(theta_1) * sin(theta_4)) * cos(theta_5) - local_coor_point[1][0] * sin(theta_1) * sin(theta_23) * sin(theta_5) -
    local_coor_point[2][0] * (sin(theta_23) * cos(theta_4) * cos(theta_5) + cos(theta_23) * sin(theta_5));
	theta_6 = atan2(ms6, mc6);

	m_angle_1_ = STDARD_ANGLE(RAD2DEGRE(theta_1));
	m_angle_2_ = STDARD_ANGLE(RAD2DEGRE(theta_2));
	m_angle_3_ = STDARD_ANGLE(RAD2DEGRE(theta_3));
	m_angle_4_ = STDARD_ANGLE(RAD2DEGRE(theta_4) + 180.0);
	m_angle_5_ = STDARD_ANGLE(-RAD2DEGRE(theta_5));
	m_angle_6_ = STDARD_ANGLE(RAD2DEGRE(theta_6) + 0.0);
}
