#include "RobotDog.h"
#include <math.h>
#include "my_debug.h"
#include "DataProgress.h"

using namespace std;

RobotDog::RobotDog()
{
	m_send_queue_ = std::make_shared<BlockQueue<std::vector<char>>>(10);

	/*leg编号初始化*/
	robot.legFR.leg_number = leg_1;
	robot.legBR.leg_number = leg_2;
	robot.legBL.leg_number = leg_3;
	robot.legFL.leg_number = leg_4;

	/*机器人body_move初始化*/
	robot.body.move_x = 0;
	robot.body.move_y = 0;
	robot.body.move_z = 0;

	/*机器人坐标系原点初始化*/
	robot.body.bodyCoorZeroPoint.x = 0;  
	robot.body.bodyCoorZeroPoint.y = 0;
	robot.body.bodyCoorZeroPoint.z = 0;

	/*机器人中心点偏移量初始化*/
	robot.body.body_centre_move_x = 0;
	robot.body.body_centre_move_y = 0;
	robot.body.body_centre_move_z = 0;

	/*四条腿顶点偏移*/
	robot.legFR.bodyPeakPoint.x = -(DOG_BODY_LENGTH/2.0 - robot.body.body_centre_move_x);
	robot.legFR.bodyPeakPoint.y = DOG_BODY_WIDE/2.0 - robot.body.body_centre_move_y;
	robot.legFR.bodyPeakPoint.z = 0;

	robot.legBR.bodyPeakPoint.x = DOG_BODY_LENGTH/2.0 + robot.body.body_centre_move_x;
	robot.legBR.bodyPeakPoint.y = DOG_BODY_WIDE/2.0 - robot.body.body_centre_move_y;
	robot.legBR.bodyPeakPoint.z = 0;

	robot.legBL.bodyPeakPoint.x = DOG_BODY_LENGTH/2.0 + robot.body.body_centre_move_x;
	robot.legBL.bodyPeakPoint.y = -(DOG_BODY_WIDE/2.0 + robot.body.body_centre_move_y);
	robot.legBL.bodyPeakPoint.z = 0;

	robot.legFL.bodyPeakPoint.x = -(DOG_BODY_LENGTH/2.0 - robot.body.body_centre_move_x);
	robot.legFL.bodyPeakPoint.y = -(DOG_BODY_WIDE/2.0 + robot.body.body_centre_move_y);
	robot.legFL.bodyPeakPoint.z = 0;

	m_send_data_.resize(24);
	m_thread_run_ = true;
	RobotDogRun();
}

RobotDog::~RobotDog()
{
	m_thread_run_ = false;
}

void RobotDog::RobotDogPoseFeedback(Pose &pose)
{
	m_robot_pose_.x = robot.body.moveVal.move_axis_x;
	m_robot_pose_.y = robot.body.moveVal.move_axis_y;
	m_robot_pose_.z = robot.body.moveVal.move_axis_z;
	m_robot_pose_.roll = robot.body.rotateVal.rot_axis_x;
	m_robot_pose_.pitch = robot.body.rotateVal.rot_axis_y;
	m_robot_pose_.yaw = robot.body.rotateVal.rot_axis_z;
	pose = m_robot_pose_;
}

void RobotDog::RobotDogCtrl(Pose posture, FootCoord foot_pos, std::vector<uint8_t> &out)
{
	robot.body.moveVal.move_axis_x = posture.x;
	robot.body.moveVal.move_axis_y = posture.y;
	robot.body.moveVal.move_axis_z = posture.z;
	robot.body.rotateVal.rot_axis_x = posture.roll;
	robot.body.rotateVal.rot_axis_y = posture.pitch;
	robot.body.rotateVal.rot_axis_z = posture.yaw;

	robot.legFR.world_coor_point.x = foot_pos.foot_FR.x;
	robot.legFR.world_coor_point.y = foot_pos.foot_FR.y;
	robot.legFR.world_coor_point.z = foot_pos.foot_FR.z;

	robot.legBR.world_coor_point.x = foot_pos.foot_BR.x;
	robot.legBR.world_coor_point.y = foot_pos.foot_BR.y;
	robot.legBR.world_coor_point.z = foot_pos.foot_BR.z;

	robot.legBL.world_coor_point.x = foot_pos.foot_BL.x;
	robot.legBL.world_coor_point.y = foot_pos.foot_BL.y;
	robot.legBL.world_coor_point.z = foot_pos.foot_BL.z;

	robot.legFL.world_coor_point.x = foot_pos.foot_FL.x;
	robot.legFL.world_coor_point.y = foot_pos.foot_FL.y;
	robot.legFL.world_coor_point.z = foot_pos.foot_FL.z;

	RobotDogProcessFlow();
	/*pos and en ctrl*/
	ServoAngleConstruct();
	out = m_send_data_;
}

void RobotDog::EN_Ctrl(bool en)
{
	RobotDogServoEnCtrl(en);
}

void RobotDog::RobotDogServoEnCtrl(bool ctrl)
{
	m_robot_enable_ = ctrl;
}

void RobotDog::RobotDogProcessFlow()
{
	/*控制body姿态，三个平移和旋转*/
	RobotDogRotMove(robot.body.rotateVal,robot.body.moveVal);

	/*正向运动学到各个关节，将世界坐标系描述的点转换到局部坐标系描述*/ 
	RobotDogInvForward(&robot.legFR);
	RobotDogInvForward(&robot.legBR);
	RobotDogInvForward(&robot.legBL);
	RobotDogInvForward(&robot.legFL);

	/*逆解，得到原始角度值*/
	LegInverseResolving(&robot.legFR);
	LegInverseResolving(&robot.legBR);
	LegInverseResolving(&robot.legBL);
	LegInverseResolving(&robot.legFL);

	/*得到五连杆机构的值*/
	GetFiveLinkAngle(&robot.legFR);
	GetFiveLinkAngle(&robot.legBR);
	GetFiveLinkAngle(&robot.legBL);
	GetFiveLinkAngle(&robot.legFL);

	/*转换到电机的角度值*/
	GetFiveLinkToMotor(&robot.legFR);
	GetFiveLinkToMotor(&robot.legBR);
	GetFiveLinkToMotor(&robot.legBL);
	GetFiveLinkToMotor(&robot.legFL);
}

void RobotDog::RobotDogRun()
{
    m_robot_thread_ = std::thread([this]
    {
        while(m_thread_run_) 
		{
			//TODO:
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));    
		}
    });
}

void RobotDog::RobotDogRotMove(BodyRot rotateVal,BodyMove moveVal)
{
	float move_body[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float ret_body_x[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float ret_body_y[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float ret_body_z[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_1[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_2[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };

	/*矩阵初始化*/
	move_body[0][3] = moveVal.move_axis_x;
	move_body[1][3] = moveVal.move_axis_y;
	move_body[2][3] = moveVal.move_axis_z;
	
	ret_body_x[1][1] = cos(rotateVal.rot_axis_x);
	ret_body_x[1][2] = -sin(rotateVal.rot_axis_x);
	ret_body_x[2][1] = sin(rotateVal.rot_axis_x);
	ret_body_x[2][2] = cos(rotateVal.rot_axis_x);
	
	ret_body_y[0][0] = cos(rotateVal.rot_axis_y);
	ret_body_y[0][2] = sin(rotateVal.rot_axis_y);
	ret_body_y[2][0] = -sin(rotateVal.rot_axis_y);
	ret_body_y[2][2] = cos(rotateVal.rot_axis_y);	
	
	ret_body_z[0][0] = cos(rotateVal.rot_axis_z);
	ret_body_z[0][1] = -sin(rotateVal.rot_axis_z);
	ret_body_z[1][0] = sin(rotateVal.rot_axis_z);
	ret_body_z[1][1] = cos(rotateVal.rot_axis_z);	
	
	MatrixCalc::MatrixMull(move_body,ret_body_x,mid_matrix_1);    
	MatrixCalc::MatrixMull(mid_matrix_1,ret_body_y,mid_matrix_2);	 
	MatrixCalc::MatrixMull(mid_matrix_2,ret_body_z,mid_matrix_1);  
	MatrixCalc::MatrixTransmit(mid_matrix_1,robot.body.postureMatrix); 
}

void RobotDog::RobotDogInvForward(Leg *leg)
{
	float rot_y[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float body_point[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_1[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_2[4][4] = { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float mid_matrix_inv[3] = { 0 }; 

	rot_y[0][0] = cos(ROT_Y);
	rot_y[0][2] = sin(ROT_Y);
	rot_y[2][0] = -sin(ROT_Y);
	rot_y[2][2] = cos(ROT_Y);

	body_point[0][3] = leg->bodyPeakPoint.x;
	body_point[1][3] = leg->bodyPeakPoint.y;

	MatrixCalc::MatrixMull(robot.body.postureMatrix, body_point, mid_matrix_1);
	MatrixCalc::MatrixMull(mid_matrix_1, rot_y, mid_matrix_2);	
	MatrixCalc::MatrixInv(mid_matrix_2, mid_matrix_1);	
	for (uint8_t i = 0; i < 3; i++)  	//将求逆后矩阵乘世界坐标系中给定的点，得到点在每条腿坐标系中的描述 
	{ 
		mid_matrix_inv[i] = mid_matrix_1[i][0] * leg->world_coor_point.x + mid_matrix_1[i][1] * leg->world_coor_point.y + mid_matrix_1[i][2] * leg->world_coor_point.z + mid_matrix_1[i][3] * 1.0;
	}	
	leg->local_coor_point.x = mid_matrix_inv[0];
	leg->local_coor_point.y = mid_matrix_inv[1];
	leg->local_coor_point.z = mid_matrix_inv[2];
}

void RobotDog::LegInverseResolving(Leg *leg)
{
	float theta_1 = 0,theta_2 = 0,theta_23 = 0,theta_3 = 0,K = 0;
	float px = 0,py = 0,pz = 0;
	float sqrt1 = 0,sqrt2 = 0;

	px = leg->local_coor_point.x;
	py = leg->local_coor_point.y;
	pz = leg->local_coor_point.z;

	sqrt1 = (px*px + py*py - DOG_D3*DOG_D3);
	if(sqrt1 < 0)
	{
		ERROR("sqrt1 < 0!");
		return;
	}
	if((leg->leg_number == leg_1) || (leg->leg_number == leg_2))
	{
		theta_1 = atan2(py, px) - atan2(DOG_D3, sqrt(sqrt1));
	}
	else
	{
		theta_1 = atan2(py, px) - atan2(DOG_D3, -sqrt(sqrt1));
	}	
    
    K = (px*px + py*py + pz*pz - DOG_A2*DOG_A2 - DOG_A3*DOG_A3 - DOG_D3*DOG_D3 - DOG_D4*DOG_D4)/(2.0 * DOG_A2);
	sqrt2 = (DOG_A3*DOG_A3 + DOG_D4*DOG_D4 - K*K);
	if(sqrt2 < 0) 
	{
		ERROR("sqrt2 < 0!");
		return;
	}	    
	if((leg->leg_number == leg_1) || (leg->leg_number == leg_2))
	{
		theta_3 = atan2(DOG_A3,DOG_D4) - atan2(K,sqrt(sqrt2));
		theta_23 = atan2((-DOG_A3 - DOG_A2 * cos(theta_3)) * pz - (cos(theta_1) * px + sin(theta_1) * py) * (DOG_D4 - DOG_A2 * sin(theta_3)), 
		(DOG_A2 * sin(theta_3) - DOG_D4) * pz + (DOG_A3 + DOG_A2 * cos(theta_3)) * (cos(theta_1) * px + sin(theta_1) * py));
	}
	else
	{
		theta_3 = atan2((-DOG_A3),DOG_D4) - atan2(K, -sqrt(sqrt2));
		theta_23 = atan2((-(-DOG_A3) - DOG_A2 * cos(theta_3)) * pz - (cos(theta_1) * px + sin(theta_1) * py) * (DOG_D4 - DOG_A2 * sin(theta_3)), 
		(DOG_A2 * sin(theta_3) - DOG_D4) * pz + ((-DOG_A3) + DOG_A2 * cos(theta_3)) * (cos(theta_1) * px + sin(theta_1) * py));
	}	

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
	leg->primitiveVal.hipAbducent = theta_1;
	leg->primitiveVal.hip = theta_2;
	leg->primitiveVal.knee = theta_3;
}

void RobotDog::GetFiveLinkToMotor(Leg *leg_kind)
{
	float mid_input[3] = { 0 };
	float mid_output[3] = { 0 };

	mid_input[0] = leg_kind->fiveLinkVal.hipAbducent;
	mid_input[1] = leg_kind->fiveLinkVal.hip;
	mid_input[2] = leg_kind->fiveLinkVal.knee;

	RobotDogAngleConvert(mid_input,mid_output,leg_kind->leg_number);

	leg_kind->motorVal.hipAbducent = mid_output[0]/PI*180.0;
	leg_kind->motorVal.hip = mid_output[1]/PI*180.0;
	leg_kind->motorVal.knee = mid_output[2]/PI*180.0;
}

void RobotDog::RobotDogAngleConvert(const float in_angle[3],float out_angle[3],RobotLeg leg)
{
	if((leg == leg_3) || (leg == leg_4))
	{
		if(leg == leg_4)
		{
			out_angle[0] = (112.7866/180.0*PI - (in_angle[0] - (180.0-56.3933)/180.0*PI)) * REDUCE_RATE;
		}
		if(leg == leg_3)
		{
			out_angle[0] = (in_angle[0] - (180.0-56.3933)/180.0*PI) * REDUCE_RATE;
		}
		if((in_angle[1]>PI) && (in_angle[1]<2.0*PI))
		{
			out_angle[1] = in_angle[1] - 225.0 / 180.0 * PI;
		}
		if((in_angle[1]>0) && (in_angle[1]<PI))
		{
			out_angle[1] = in_angle[1] + 135.0 / 180.0 * PI;
		}
		out_angle[2] = in_angle[2] - 135.0 / 180.0 * PI;	
	}
	if((leg == leg_1) || (leg == leg_2))
	{
		if(leg == leg_1)
		{
			if(in_angle[0] > 270.0/180.0*PI)
			{
				out_angle[0] = (112.7866/180.0*PI - (in_angle[0] - (360-56.393)/180.0*PI)) * REDUCE_RATE;
			}
			else
			{
				out_angle[0] = (112.7866/180.0*PI - (in_angle[0] + 56.393/180.0*PI)) * REDUCE_RATE;
			}	
		}
		if(leg == leg_2)
		{
			if(in_angle[0] > 270.0/180.0*PI)
			{
				out_angle[0] = (in_angle[0] - (360-56.393)/180.0*PI) * REDUCE_RATE;
			}
			else
			{
				out_angle[0] = (in_angle[0] + 56.393/180.0*PI) * REDUCE_RATE;
			}	
		}
		out_angle[1] = in_angle[1] - 135.0 / 180.0 * PI;
		if((in_angle[2]>PI) && (in_angle[2]<2.0*PI))
		{
			out_angle[2] = in_angle[2] - 225.0 / 180.0 * PI;
		}
		if((in_angle[2]>0) && (in_angle[2]<PI))
		{
			out_angle[2] = in_angle[2] + 135.0 / 180.0 * PI;
		}
	}
}

void RobotDog::RobotDogConvertToMotorAngle(const float in_angle[3],float out_angle[3],RobotLeg leg )
{
	float test_out_an = 0;
	float test_sqrt3 = 0;
	float c2 = 0,s2 = 0,x = 0,y = 0,l1 = 0,l2 = 0,theta_1 = 0,theta_2 = 0,k1 = 0,k2 = 0;
	float mid_in_angle[3] = { 0 };
	for(uint8_t i=0;i<3;i++)
	{
		mid_in_angle[i] = in_angle[i];
	}
	
	float move_matrix_1[4][4] =    { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float move_matrix_2[4][4] =    { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float move_matrix_3[4][4] =    { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_matrix_1[4][4] =     { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };
	float rot_matrix_2[4][4] =     { { 1,0,0,0 },{ 0,1,0,0 },{ 0,0,1,0 },{ 0,0,0,1 } };

	float mid_matrix_1[4][4];
	float mid_matrix_2[4][4];
	float mid_matrix_3[4][4];
	float mid_matrix_4[4][4];
		
	if((leg == leg_3) || (leg == leg_4))
	{
		move_matrix_1[0][3] = DOG_A;
		mid_in_angle[1] = mid_in_angle[1] + 0.5 * PI;	
		mid_in_angle[2] = mid_in_angle[2] + 0.5 * PI;	
	}	
	if((leg == leg_1) || (leg == leg_2))
	{
		move_matrix_1[0][3] = -DOG_A;
		mid_in_angle[1] = mid_in_angle[1] - 0.5 * PI;	
		mid_in_angle[2] = mid_in_angle[2] + 0.5 * PI;
	}
	if(mid_in_angle[1] > 2.0 * PI)
	{
		mid_in_angle[1] = mid_in_angle[1] - 2.0 * PI;
	}
	if(mid_in_angle[2] > 2.0 * PI)
	{
		mid_in_angle[2] = mid_in_angle[2] - 2.0 * PI;
	}

	move_matrix_1[1][3] = 0;
	move_matrix_1[2][3] = 0;
		
	move_matrix_2[0][3] = DOG_L2;
	move_matrix_2[1][3] = 0;
	move_matrix_2[2][3] = 0;

	move_matrix_3[0][3] = DOG_MID_L3;
	move_matrix_3[1][3] = 0;
	move_matrix_3[2][3] = 0;
		
	rot_matrix_1[0][0] = cos(mid_in_angle[1]);
	rot_matrix_1[0][1] = -sin(mid_in_angle[1]);
	rot_matrix_1[1][0] = sin(mid_in_angle[1]);
	rot_matrix_1[1][1] = cos(mid_in_angle[1]);	

	rot_matrix_2[0][0] = cos(mid_in_angle[2]);
	rot_matrix_2[0][1] = -sin(mid_in_angle[2]);
	rot_matrix_2[1][0] = sin(mid_in_angle[2]);
	rot_matrix_2[1][1] = cos(mid_in_angle[2]);	

	MatrixCalc::MatrixMull(move_matrix_1,rot_matrix_1,mid_matrix_1);
	MatrixCalc::MatrixMull(mid_matrix_1,move_matrix_2,mid_matrix_2);
	MatrixCalc::MatrixMull(mid_matrix_2,rot_matrix_2,mid_matrix_3);
	MatrixCalc::MatrixMull(mid_matrix_3,move_matrix_3,mid_matrix_4);
	
	l1 = DOG_LEFT_L1;
	l2 = DOG_LEFT_L2;
	x = mid_matrix_4[0][3];
	y = mid_matrix_4[1][3];
	
	c2 = (x*x + y*y - l1*l1 - l2*l2) / (2.0 * l1 * l2);
	
	test_sqrt3 = (1.0 - c2*c2);
	if(test_sqrt3 < 0.1)
	{
		return;
	}
	
	if((leg == leg_3) || (leg == leg_4))	
	{
		s2 = sqrt(test_sqrt3);
	}
	if((leg == leg_1) || (leg == leg_2))	
	{
		s2 = -sqrt(test_sqrt3);
	}
	k1 = l1 + l2 * c2;
	k2 = l2 * s2;
	theta_1 = (atan2(y,x) - atan2(k2,k1));
	/*以下三个角度为相对于‘世界坐标系的角度’*/ 
	out_angle[0] = mid_in_angle[0];
	out_angle[1] = mid_in_angle[1];
	if(theta_1 < 0)
	{
		out_angle[2] = 2.0*PI + theta_1;
		test_out_an = 2.0*PI + theta_1;
	}
	else
	{
		out_angle[2] = theta_1;
		test_out_an = theta_1;
	}
}

void RobotDog::GetFiveLinkAngle(Leg *leg_kind)
{
	float mid_input[3] = { 0 };
	float mid_output[3] = { 0 };

	mid_output[0] = leg_kind->fiveLinkVal.hipAbducent;
	mid_output[1] = leg_kind->fiveLinkVal.hip;
	mid_output[2] = leg_kind->fiveLinkVal.knee;

	mid_input[0] = leg_kind->primitiveVal.hipAbducent;
	mid_input[1] = leg_kind->primitiveVal.hip;
	mid_input[2] = leg_kind->primitiveVal.knee;

	RobotDogConvertToMotorAngle(mid_input,mid_output,leg_kind->leg_number);

	leg_kind->fiveLinkVal.hipAbducent = mid_output[0];
	leg_kind->fiveLinkVal.hip = mid_output[1];
	leg_kind->fiveLinkVal.knee = mid_output[2];
}

void RobotDog::GetAngleAndPost(float angle[4][3],float pos[2][3])
{
	for(uint16_t i = 0; i < 4; i++)
	{
		if(i == 0)
		{
			angle[i][0] = RAD2DEGRE(robot.legFR.primitiveVal.hipAbducent);
			angle[i][1] = RAD2DEGRE(robot.legFR.primitiveVal.hip);
			angle[i][2] = RAD2DEGRE(robot.legFR.primitiveVal.knee);
		}
		if(i == 1)
		{
			angle[i][0] = RAD2DEGRE(robot.legBR.primitiveVal.hipAbducent);
			angle[i][1] = RAD2DEGRE(robot.legBR.primitiveVal.hip);
			angle[i][2] = RAD2DEGRE(robot.legBR.primitiveVal.knee);
		}
		if(i == 2)
		{
			angle[i][0] = RAD2DEGRE(robot.legBL.primitiveVal.hipAbducent);
			angle[i][1] = RAD2DEGRE(robot.legBL.primitiveVal.hip);
			angle[i][2] = RAD2DEGRE(robot.legBL.primitiveVal.knee);
		}
		if(i == 3)
		{
			angle[i][0] = RAD2DEGRE(robot.legFL.primitiveVal.hipAbducent);
			angle[i][1] = RAD2DEGRE(robot.legFL.primitiveVal.hip);
			angle[i][2] = RAD2DEGRE(robot.legFL.primitiveVal.knee);
		}	
	}

	pos[0][0] = robot.body.moveVal.move_axis_x;
	pos[0][1] = robot.body.moveVal.move_axis_y;
	pos[0][2] = robot.body.moveVal.move_axis_z;
	
	pos[1][0] = RAD2DEGRE(robot.body.rotateVal.rot_axis_x);
	pos[1][1] = RAD2DEGRE(robot.body.rotateVal.rot_axis_y);
	pos[1][2] = RAD2DEGRE(robot.body.rotateVal.rot_axis_z);
}

void RobotDog::ServoAngleConstruct()
{
	int16_t temp_data = 0;
	temp_data = (int16_t)((robot.legFR.motorVal.hipAbducent) * 100.0);
	m_send_data_[0] = (temp_data >> 8) & 0xff;
	m_send_data_[1] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legFR.motorVal.hip) * 100.0);
	m_send_data_[2] = (temp_data >> 8) & 0xff;
	m_send_data_[3] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legFR.motorVal.knee) * 100.0);
	m_send_data_[4] = (temp_data >> 8) & 0xff;
	m_send_data_[5] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBR.motorVal.hipAbducent) * 100.0);
	m_send_data_[6] = (temp_data >> 8) & 0xff;
	m_send_data_[7] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBR.motorVal.hip) * 100.0);
	m_send_data_[8] = (temp_data >> 8) & 0xff;
	m_send_data_[9] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBR.motorVal.knee) * 100.0);
	m_send_data_[10] = (temp_data >> 8) & 0xff;
	m_send_data_[11] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBL.motorVal.hipAbducent) * 100.0);
	m_send_data_[12] = (temp_data >> 8) & 0xff;
	m_send_data_[13] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBL.motorVal.hip) * 100.0);
	m_send_data_[14] = (temp_data >> 8) & 0xff;
	m_send_data_[15] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legBL.motorVal.knee) * 100.0);
	m_send_data_[16] = (temp_data >> 8) & 0xff;
	m_send_data_[17] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legFL.motorVal.hipAbducent) * 100.0);
	m_send_data_[18] = (temp_data >> 8) & 0xff;
	m_send_data_[19] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legFL.motorVal.hip) * 100.0);
	m_send_data_[20] = (temp_data >> 8) & 0xff;
	m_send_data_[21] = (temp_data) & 0xff;

	temp_data = (int16_t)((robot.legFL.motorVal.knee) * 100.0);
	m_send_data_[22] = (temp_data >> 8) & 0xff;
	m_send_data_[23] = (temp_data) & 0xff;
}
