#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "Robot.h"
#include "my_debug.h"
#include "DataProgress.h"
#include "socket_paint.h"

Robot::Robot()
{
    SocketPaint::Instance().Init();
    DataProgress::Instance().RegistArmAngleCallback(
        std::bind(&Robot::ArmAngleFB, this, std::placeholders::_1));
    DataProgress::Instance().RegistIMU_PosCallback(
        std::bind(&Robot::IMU_PosFB, this, std::placeholders::_1));

    m_pid_controller_ = std::make_shared<intertialController>();
    m_vir_executer_ = std::make_shared<virtralExecuter>();
    m_pid_controller_->RegistPosGetCallback(
        std::bind(&virtralExecuter::VirtualExcuterGetPos, m_vir_executer_));

    ParaInit();
    m_thread_run_ = true;
    sem_init(&m_sem_,0,0);
    robotRun();
}

Robot::~Robot()
{
    m_thread_run_ = false;
}

void Robot::ParaInit()
{
	/*四条腿世界坐标系点初始化*/
    m_foot_pos_.foot_FR.x = GAIT_FIRST_POINT_1_X;
	m_foot_pos_.foot_FR.y = GAIT_FIRST_POINT_1_Y;
	m_foot_pos_.foot_FR.z = GAIT_FIRST_POINT_1_Z;

	m_foot_pos_.foot_BR.x = GAIT_FIRST_POINT_2_X;
	m_foot_pos_.foot_BR.y = GAIT_FIRST_POINT_2_Y;
	m_foot_pos_.foot_BR.z = GAIT_FIRST_POINT_2_Z;

	m_foot_pos_.foot_BL.x = GAIT_FIRST_POINT_3_X;
	m_foot_pos_.foot_BL.y = GAIT_FIRST_POINT_3_Y;
	m_foot_pos_.foot_BL.z = GAIT_FIRST_POINT_3_Z;

	m_foot_pos_.foot_FL.x = GAIT_FIRST_POINT_4_X;
	m_foot_pos_.foot_FL.y = GAIT_FIRST_POINT_4_Y;
	m_foot_pos_.foot_FL.z = GAIT_FIRST_POINT_4_Z;

	/*机器人body偏移量初始化*/
    m_dog_posture_.x = FIRST_BODY_POS_X;
    m_dog_posture_.y = FIRST_BODY_POS_Y;
    m_dog_posture_.z = FIRST_BODY_POS_Z;
    m_dog_posture_.roll = 0;
    m_dog_posture_.pitch = 0;
    m_dog_posture_.yaw = 0;

	/*filter init*/
    m_dog_filter_mx_.KalmanFilterInit(m_dog_posture_.x, m_dog_kalman_Q_, m_dog_kalman_R_);
	m_dog_filter_my_.KalmanFilterInit(m_dog_posture_.y, m_dog_kalman_Q_, m_dog_kalman_R_);
	m_dog_filter_mz_.KalmanFilterInit(m_dog_posture_.z, m_dog_kalman_Q_, m_dog_kalman_R_);
	m_dog_filter_rx_.KalmanFilterInit(m_dog_posture_.roll, m_dog_kalman_Q_, m_dog_kalman_R_);
	m_dog_filter_ry_.KalmanFilterInit(m_dog_posture_.pitch, m_dog_kalman_Q_, m_dog_kalman_R_);
	m_dog_filter_rz_.KalmanFilterInit(m_dog_posture_.yaw, m_dog_kalman_Q_, m_dog_kalman_R_);

	m_arm_filter_mx_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);
    m_arm_filter_my_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);
	m_arm_filter_mz_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);
	m_arm_filter_rx_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);
	m_arm_filter_ry_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);
	m_arm_filter_rz_.KalmanFilterInit(0, m_arm_kalman_Q_, m_arm_kalman_R_);

    m_imu_ctrl_.KalmanFilterInit(0, 0.1, 2.0);

    /*CubicInterpolation init*/
    m_arm_axis_.resize(7);
    for(auto &a : m_arm_axis_)
    {
        a.interpolater = std::make_shared<CubicInterpolation>(INTERPOLATION_PERIOD);
    }

    m_arm_axis_[0].para_k = -1;
    m_arm_axis_[1].para_k = 1;
    m_arm_axis_[2].para_k = 1;
    m_arm_axis_[3].para_k = -1;
    m_arm_axis_[4].para_k = 1;
    m_arm_axis_[5].para_k = -1;
    m_arm_axis_[6].para_k = -1;

    m_arm_axis_[0].para_b = 121;
    m_arm_axis_[1].para_b = -174;
    m_arm_axis_[2].para_b = 77;
    m_arm_axis_[3].para_b = 117;
    m_arm_axis_[4].para_b = 113;
    m_arm_axis_[5].para_b = 117;
    m_arm_axis_[6].para_b = 210;
}

void Robot::RegistRobotArmImpl(std::shared_ptr<ArmCtrlBase> arm_ptr)
{
    m_arm_ctr_ = arm_ptr;
}

void Robot::RegistRobotDogImpl(std::shared_ptr<DogCtrlBase> dog_ptr)
{
    m_dog_ctr_ = dog_ptr;
}

void Robot::ConstructAndSendFrame(uint8_t function_code, uint8_t data_len, std::vector<uint8_t> data)
{
    m_send_data_.check_sum = 0; 
    m_send_data_.head = UART_DATA_HEAD;
    m_send_data_.function_code = function_code;
    m_send_data_.data_len = data_len;
    m_send_data_.data = data;
    m_send_data_.check_sum += m_send_data_.function_code;
    m_send_data_.check_sum += m_send_data_.data_len;
    for(auto &a : m_send_data_.data)
    {
        m_send_data_.check_sum += a;
    }
    DataProgress::Instance().SerialSend(m_send_data_); 
}

bool Robot::RobotDataReq(DataReqModel &req)
{
    for(int i = 0; i < req.retry_times; i++)
    {
        ConstructAndSendFrame(req.func_code, req.data_len, req.data); 
        if(req.sem.SemTimedWait(req.time_out_ms))
        {
            return true;
        }
    }
	return false;
}

void Robot::ArmAngleFB(ArmAngleFeedBack &angle)
{
    m_arm_first_angle_ = angle;
    m_arm_res_flg_ = true;
    m_angle_req_.sem.SemPost();
}

void Robot::IMU_PosFB(coord_point &pos)
{
    m_imu_pos_ = pos;
    m_imu_pos_req_.sem.SemPost();
}

void Robot::IMU_PosReq()
{
    std::vector<uint8_t> tmp(2, 0);

    m_imu_pos_req_.func_code = TYPE_IMU_POS_REQ; 
    m_imu_pos_req_.data = tmp; 
    m_imu_pos_req_.data_len = 2; 
    m_imu_pos_req_.retry_times = 5; 
    m_imu_pos_req_.time_out_ms = 2; 
    if(!RobotDataReq(m_imu_pos_req_)) 
    {
        printf("imu pos req failed!\n");
    }
}

void Robot::ArmDisable()
{
    std::vector<uint8_t> tmp(2, 0);

    m_angle_req_.func_code = TYPE_ARM_FIRST_ANGLE_REQ; 
    m_angle_req_.data = tmp; 
    m_angle_req_.data_len = 2; 
    m_angle_req_.retry_times = 5; 
    m_angle_req_.time_out_ms = 200; 
    if(RobotDataReq(m_angle_req_)) 
    {
        printf("arm disable success!\n");
    }
    else
    {
        printf("arm disable failed!\n");
    }
}

void Robot::ArmStretch()
{
    std::vector<uint8_t> tmp(2, 0);
    m_arm_res_flg_ = false;

    m_angle_req_.func_code = TYPE_ARM_FIRST_ANGLE_REQ;
    m_angle_req_.data = tmp; 
    m_angle_req_.data_len = 2; 
    m_angle_req_.retry_times = 5; 
    m_angle_req_.time_out_ms = 200; 
    if(RobotDataReq(m_angle_req_)) 
    {
        printf("arm first angle req success!\n");
    }
    else
    {
        printf("arm first angle req false!\n");
    }

    printf("start to stretch!\n");
    m_arm_axis_[0].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_1;
    m_arm_axis_[0].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[0].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[0].interpolater->m_cubic_end.position = 121;
    m_arm_axis_[0].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[0].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[0].interpolater->CubicInterpolationBegin();

    m_arm_axis_[1].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_2;
    m_arm_axis_[1].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[1].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[1].interpolater->m_cubic_end.position = 84;
    m_arm_axis_[1].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[1].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[1].interpolater->CubicInterpolationBegin();

    m_arm_axis_[2].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_3;
    m_arm_axis_[2].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[2].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[2].interpolater->m_cubic_end.position = 129;
    m_arm_axis_[2].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[2].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[2].interpolater->CubicInterpolationBegin();

    m_arm_axis_[3].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_4;
    m_arm_axis_[3].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[3].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[3].interpolater->m_cubic_end.position = 117;
    m_arm_axis_[3].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[3].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[3].interpolater->CubicInterpolationBegin();

    m_arm_axis_[4].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_5;
    m_arm_axis_[4].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[4].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[4].interpolater->m_cubic_end.position = 72;
    m_arm_axis_[4].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[4].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[4].interpolater->CubicInterpolationBegin();

    m_arm_axis_[5].interpolater->m_cubic_start.position = m_arm_first_angle_.axis_angle.axis_6;
    m_arm_axis_[5].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[5].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[5].interpolater->m_cubic_end.position = 117;
    m_arm_axis_[5].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[5].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[5].interpolater->CubicInterpolationBegin();

    m_arm_axis_[6].interpolater->m_cubic_start.position = m_arm_first_angle_.claw_angle;
    m_arm_axis_[6].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[6].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[6].interpolater->m_cubic_end.position = 210;
    m_arm_axis_[6].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[6].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[6].interpolater->CubicInterpolationBegin();

    tmp.clear();
    tmp.assign(14, 0);
    double pos = 0;
    uint16_t tem_pos = 0;

    for(int i = 0; i < (int)(INTERPOLATION_DURATION / INTERPOLATION_PERIOD); i++)
    {
        for(int j = 0; j < 7; j++)
        {
            if(m_arm_axis_[j].interpolater->CubicInterpolationRuner(pos) == true)
            {
                tem_pos = (uint16_t)(pos * 100.0);
                tmp[j * 2] = (tem_pos >> 8) & 0x00ff; 
                tmp[j * 2 + 1] = (tem_pos) & 0x00ff; 
            }
        }
        ConstructAndSendFrame(TYPE_ARM_POS_CTR, 14, tmp);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }
    printf("arm stretch complite!\n");
}

void Robot::ArmContract()
{
    std::vector<uint8_t> tmp(14, 0);

    printf("arm start to contract!\n");
    m_arm_axis_[0].interpolater->m_cubic_start.position = m_arm_axis_[0].angle;
    m_arm_axis_[0].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[0].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[0].interpolater->m_cubic_end.position = 121;
    m_arm_axis_[0].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[0].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[0].interpolater->CubicInterpolationBegin();

    m_arm_axis_[1].interpolater->m_cubic_start.position = m_arm_axis_[1].angle;
    m_arm_axis_[1].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[1].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[1].interpolater->m_cubic_end.position = 6;
    m_arm_axis_[1].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[1].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[1].interpolater->CubicInterpolationBegin();

    m_arm_axis_[2].interpolater->m_cubic_start.position = m_arm_axis_[2].angle;
    m_arm_axis_[2].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[2].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[2].interpolater->m_cubic_end.position = 167;
    m_arm_axis_[2].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[2].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[2].interpolater->CubicInterpolationBegin();

    m_arm_axis_[3].interpolater->m_cubic_start.position = m_arm_axis_[3].angle;
    m_arm_axis_[3].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[3].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[3].interpolater->m_cubic_end.position = 117;
    m_arm_axis_[3].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[3].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[3].interpolater->CubicInterpolationBegin();

    m_arm_axis_[4].interpolater->m_cubic_start.position = m_arm_axis_[4].angle;
    m_arm_axis_[4].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[4].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[4].interpolater->m_cubic_end.position = 113;
    m_arm_axis_[4].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[4].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[4].interpolater->CubicInterpolationBegin();

    m_arm_axis_[5].interpolater->m_cubic_start.position = m_arm_axis_[5].angle;
    m_arm_axis_[5].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[5].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[5].interpolater->m_cubic_end.position = 117;
    m_arm_axis_[5].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[5].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[5].interpolater->CubicInterpolationBegin();

    m_arm_axis_[6].interpolater->m_cubic_start.position = m_arm_axis_[6].angle;
    m_arm_axis_[6].interpolater->m_cubic_start.velocity = 0;
    m_arm_axis_[6].interpolater->m_cubic_start.time = 0;
    m_arm_axis_[6].interpolater->m_cubic_end.position = 210;
    m_arm_axis_[6].interpolater->m_cubic_end.velocity = 0;
    m_arm_axis_[6].interpolater->m_cubic_end.time = INTERPOLATION_DURATION;
    m_arm_axis_[6].interpolater->CubicInterpolationBegin();

    tmp.clear();
    tmp.assign(14, 0);
    double pos = 0;
    uint16_t tem_pos = 0;

    /*0轴  3轴  5轴  6轴 先收回*/
    for(int i = 0; i < (int)(INTERPOLATION_DURATION / INTERPOLATION_PERIOD); i++)  
    {
        if(m_arm_axis_[0].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[0] = (tem_pos >> 8) & 0x00ff; 
            tmp[1] = (tem_pos) & 0x00ff; 
            m_arm_axis_[0].angle = pos; 
        }
        tem_pos = (uint16_t)(m_arm_axis_[1].angle * 100.0);  //1轴角度不变
        tmp[2] = (tem_pos >> 8) & 0x00ff; 
        tmp[3] = (tem_pos) & 0x00ff; 
        tem_pos = (uint16_t)(m_arm_axis_[2].angle * 100.0);  //2轴角度不变
        tmp[4] = (tem_pos >> 8) & 0x00ff; 
        tmp[5] = (tem_pos) & 0x00ff; 
        if(m_arm_axis_[3].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[6] = (tem_pos >> 8) & 0x00ff; 
            tmp[7] = (tem_pos) & 0x00ff; 
            m_arm_axis_[3].angle = pos; 
        }
        tem_pos = (uint16_t)(m_arm_axis_[4].angle * 100.0);  //4轴角度不变
        tmp[8] = (tem_pos >> 8) & 0x00ff; 
        tmp[9] = (tem_pos) & 0x00ff; 
        if(m_arm_axis_[5].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[10] = (tem_pos >> 8) & 0x00ff; 
            tmp[11] = (tem_pos) & 0x00ff; 
            m_arm_axis_[5].angle = pos; 
        }
        if(m_arm_axis_[6].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[12] = (tem_pos >> 8) & 0x00ff; 
            tmp[13] = (tem_pos) & 0x00ff; 
            m_arm_axis_[6].angle = pos; 
        }

        ConstructAndSendFrame(TYPE_ARM_POS_CTR, 14, tmp); 
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }

    /*再收回 1 2 4 轴*/
    for(int i = 0; i < (int)(INTERPOLATION_DURATION / INTERPOLATION_PERIOD); i++)  //0轴  3轴  先收回
    {
        tem_pos = (uint16_t)(m_arm_axis_[0].angle * 100.0); 
        tmp[0] = (tem_pos >> 8) & 0x00ff; 
        tmp[1] = (tem_pos) & 0x00ff; 
        if(m_arm_axis_[1].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[2] = (tem_pos >> 8) & 0x00ff; 
            tmp[3] = (tem_pos) & 0x00ff; 
        }
        if(m_arm_axis_[2].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[4] = (tem_pos >> 8) & 0x00ff; 
            tmp[5] = (tem_pos) & 0x00ff; 
        }
        tem_pos = (uint16_t)(m_arm_axis_[3].angle * 100.0);  //3轴角度不变
        tmp[6] = (tem_pos >> 8) & 0x00ff; 
        tmp[7] = (tem_pos) & 0x00ff; 
        if(m_arm_axis_[4].interpolater->CubicInterpolationRuner(pos) == true)
        {
            tem_pos = (uint16_t)(pos * 100.0);
            tmp[8] = (tem_pos >> 8) & 0x00ff; 
            tmp[9] = (tem_pos) & 0x00ff; 
        }
        tem_pos = (uint16_t)(m_arm_axis_[5].angle * 100.0);  //5轴角度不变
        tmp[10] = (tem_pos >> 8) & 0x00ff; 
        tmp[11] = (tem_pos) & 0x00ff; 
        tem_pos = (uint16_t)(m_arm_axis_[6].angle * 100.0);  //6轴角度不变
        tmp[12] = (tem_pos >> 8) & 0x00ff; 
        tmp[13] = (tem_pos) & 0x00ff; 

        ConstructAndSendFrame(TYPE_ARM_POS_CTR, 14, tmp); 
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }
    printf("arm contract complite!\n");
}

void Robot::ArmCtrlInit()
{
    m_arm_point_.x = 78.05 - 150;
    m_arm_point_.y = 375 - 0;
    m_arm_point_.z = 307.35 + 100;
    m_arm_point_.roll = DEGRE2RAD(0);
    m_arm_point_.pitch = DEGRE2RAD(0);
    m_arm_point_.yaw = DEGRE2RAD(0);

    m_axis_ctrl_quantity_.move_x = 0;
    m_axis_ctrl_quantity_.move_y = 0;
    m_axis_ctrl_quantity_.move_z = 0;
    m_axis_ctrl_quantity_.rotate_x = 0;
    m_axis_ctrl_quantity_.rotate_y = 0;
    m_axis_ctrl_quantity_.rotate_z = 0;
    m_axis_ctrl_quantity_.axis_claw = 0;

    IMU_PosReq();
    m_light_first_val_ = m_imu_pos_.y;
    m_imu_pos_.y = 375;
    m_vir_executer_->virtralExecuterParaInit(375);
    m_movez_alarm_flg_ = true;
}

void Robot::ArmCtrl()
{
    AxisAngle tmp_angle;
    uint16_t tem_pos = 0;
    std::vector<uint8_t> tmp(14 + 6, 0);  

    if(m_arm_ctr_ != nullptr)
    {
        m_arm_ctr_->ArmCoordinateMode(m_arm_point_, tmp_angle);
        /*零位偏差，以及方向，比例系数校正*/
        m_arm_axis_[0].angle = AngleConvert(m_arm_axis_[0], tmp_angle.axis_1);
        m_arm_axis_[1].angle = AngleConvert(m_arm_axis_[1], tmp_angle.axis_2);
        m_arm_axis_[2].angle = AngleConvert(m_arm_axis_[2], tmp_angle.axis_3);
        m_arm_axis_[3].angle = AngleConvert(m_arm_axis_[3], tmp_angle.axis_4);
        m_arm_axis_[4].angle = AngleConvert(m_arm_axis_[4], tmp_angle.axis_5);
        m_arm_axis_[5].angle = AngleConvert(m_arm_axis_[5], tmp_angle.axis_6);
        m_arm_axis_[6].angle = AngleConvert(m_arm_axis_[6], m_axis_ctrl_quantity_.axis_claw); 

        for(int i = 0; i < 7; i++) 
        { 
            tem_pos = (uint16_t)(m_arm_axis_[i].angle * 100.0);
            tmp[i * 2] = (tem_pos >> 8) & 0x00ff; 
            tmp[i * 2 + 1] = (tem_pos) & 0x00ff; 
        }
        //增加3个末端位置
        tem_tail_pos = (int16_t)(m_arm_point_.x * 10.0);
        tmp[14] = (tem_tail_pos >> 8) & 0x00ff;
        tmp[15] = (tem_tail_pos) & 0x00ff;

        tem_tail_pos = (int16_t)(m_arm_point_.y * 10.0);
        tmp[16] = (tem_tail_pos >> 8) & 0x00ff;
        tmp[17] = (tem_tail_pos) & 0x00ff;

        tem_tail_pos = (int16_t)(m_arm_point_.z * 10.0);
        tmp[18] = (tem_tail_pos >> 8) & 0x00ff;
        tmp[19] = (tem_tail_pos) & 0x00ff;

        ConstructAndSendFrame(TYPE_ARM_POS_CTR, 14 + 6, tmp);
    }
}

float Robot::AngleConvert(ArmAxis &axis, float in)
{
    float out = 0;

    out = in * axis.para_k + axis.para_b;
    if(out > 360)
    {
        out -= 360;
    }
    if(out < 0)
    {
        out += 360;
    }
    return out;
}

void Robot::RobotDogControl()
{
    if(m_dog_ctr_ != nullptr)
    {
        m_dog_ctr_->RobotDogCtrl(m_dog_posture_, m_foot_pos_, m_servo_data_);
        ConstructAndSendFrame(TYPE_DOG_POS_CTR, 24, m_servo_data_);
    }
}

void Robot::EnableJudgment()
{
    RemoteController::Instance().GetSwitchStatus(m_switch_ch_);
    if(m_switch_ch_.switch_D == SwitchStatus::on) 
    {
        m_is_enable_ = true;
    }
    else
    {
        m_is_enable_ = false;
    }
}

void Robot::AnalogChannelMapping()
{
    float ch1_val = 0, ch2_val = 0, ch3_val = 0, ch4_val = 0, ch9_val = 0, ch10_val = 0;

    AllAnalogChannel channel_data;
    RemoteController::Instance().GetAnalogChannel(&channel_data); 

    ch1_val = channel_data.ch_1.interpolate_para.out;
    ch2_val = channel_data.ch_2.interpolate_para.out;
    ch3_val = channel_data.ch_3.interpolate_para.out;
    ch4_val = channel_data.ch_4.interpolate_para.out;
    ch9_val = channel_data.ch_9.interpolate_para.out;
    ch10_val = channel_data.ch_10.interpolate_para.out;

    if(m_switch_ch_.switch_B == SwitchStatus::off)
    {
        m_analog_ch_map_ = AnalogChannelOn::ChannelOnDog;

        m_dog_posture_.y = m_dog_filter_my_.KalmanFilterFun(FIRST_BODY_POS_Y + (ch4_val - 500) / 5.0);
        m_dog_posture_.z = m_dog_filter_mz_.KalmanFilterFun(FIRST_BODY_POS_Z + (ch9_val - 500) / 11.0);

        m_dog_posture_.roll = -m_dog_filter_rx_.KalmanFilterFun(0 + DEGRE2RAD((ch10_val - 500) / 17.0)); 
        m_dog_posture_.pitch = -m_dog_filter_ry_.KalmanFilterFun(0 + DEGRE2RAD((ch2_val - 500) / 25.0));  
        m_dog_posture_.yaw = -m_dog_filter_rz_.KalmanFilterFun(0 + DEGRE2RAD((ch1_val - 500) / 25.0)); 
        
        // SocketPaint::Instance().DrawPoint(1, 0, (int16_t)(m_dog_posture_.roll * 500));
        // SocketPaint::Instance().DrawPoint(2, 60, (int16_t)(m_dog_posture_.pitch * 500));
        // SocketPaint::Instance().DrawPoint(3, 120, (int16_t)(m_dog_posture_.yaw * 500));
        // SocketPaint::Instance().DrawPoint(4, 180, (int16_t)m_dog_posture_.y - FIRST_BODY_POS_Y);
        // SocketPaint::Instance().DrawPoint(5, 240, (int16_t)m_dog_posture_.z - FIRST_BODY_POS_Z);
    }  
    else 
    {
        m_analog_ch_map_ = AnalogChannelOn::ChannelOnArm;

        m_axis_ctrl_quantity_.move_y += RemoteController::Instance().ChannelPreProcess(ch1_val, 20) / 100.0; 
        m_axis_ctrl_quantity_.move_x += RemoteController::Instance().ChannelPreProcess(ch2_val, 20) / 100.0; 
        m_arm_point_.y = 375 + m_axis_ctrl_quantity_.move_y;
        m_arm_point_.x = (78.05 - 150) + (-m_axis_ctrl_quantity_.move_x); 
        
        /*世界坐标系中的绕轴旋转，注意与机械臂末端坐标系方向不一致！*/
        m_axis_ctrl_quantity_.rotate_x += RemoteController::Instance().ChannelPreProcess(ch4_val, 20) / 75.0; 
        m_arm_point_.yaw = DEGRE2RAD(m_axis_ctrl_quantity_.rotate_x);
        if(m_switch_ch_.switch_C == SwitchStatus::off)
        {
            m_axis_ctrl_quantity_.rotate_y += RemoteController::Instance().ChannelPreProcess(ch9_val, 20) / 170.0; 
        }
        else
        {
            m_axis_ctrl_quantity_.axis_claw += RemoteController::Instance().ChannelPreProcess(ch9_val, 20) / 170.0; 
            if(m_axis_ctrl_quantity_.axis_claw < 0)
            {
                m_axis_ctrl_quantity_.axis_claw = 0;
            }
            if(m_axis_ctrl_quantity_.axis_claw > 90)
            {
                m_axis_ctrl_quantity_.axis_claw = 90;
            }
        }
        
        m_arm_point_.pitch = DEGRE2RAD(-m_axis_ctrl_quantity_.rotate_y);
        m_axis_ctrl_quantity_.rotate_z += RemoteController::Instance().ChannelPreProcess(ch10_val, 20) / 170.0; 
        m_arm_point_.roll = DEGRE2RAD(m_axis_ctrl_quantity_.rotate_z);

        if(ABS(RemoteController::Instance().ChannelPreProcess(ch3_val, 100)) < 0.1)
        {
            m_movez_alarm_flg_ = false;
        }
        if(m_movez_alarm_flg_ == false)
        {
            m_axis_ctrl_quantity_.move_z += RemoteController::Instance().ChannelPreProcess(ch3_val, 100) / 100.0; 
            m_arm_point_.z = (307.35 + 100) + (m_axis_ctrl_quantity_.move_z);
        }
    }
}

void Robot::ArmStateMachine()
{
    bool key = false;
    static RobotOperationStatus arm_status = RobotOperationStatus::ArmReset;

    RemoteController::Instance().GetLongPressStatus(key);
    switch(arm_status)
    {
        case RobotOperationStatus::ArmStretch:
            ArmStretch(); //arm伸出
            arm_status = RobotOperationStatus::ArmInCtrl;
            ArmCtrlInit();
            printf("robot arm to ctrl!\n");
        break;

        case RobotOperationStatus::ArmInCtrl:
            ArmCtrl();
            if(key)
            {
                printf("robot arm contract!\n");
                arm_status = RobotOperationStatus::ArmContract;
                RemoteController::Instance().ClearKeyStatus(); 
            }        
        break;

        case RobotOperationStatus::ArmContract:
            ArmContract();
            ArmDisable(); //收回完成，失能电机
            arm_status = RobotOperationStatus::ArmReset;
            printf("robot arm reset!\n");
        break;

        case RobotOperationStatus::ArmReset:
            if(key)
            {
                printf("robot arm stretch!\n");
                arm_status = RobotOperationStatus::ArmStretch;
                RemoteController::Instance().ClearKeyStatus(); 
            }
        break;    

        default:
        break;
    }
}

void Robot::robotRun()
{
    static int count = 0;

    m_thread_ = std::thread([this]
    {
        while(m_thread_run_)
		{
            EnableJudgment();
            if(m_is_enable_)
            {
                count++;
                if(count > 10)
                {
                    count = 10;

                    AnalogChannelMapping();
                    ArmStateMachine();
                    RobotDogControl();
                }
            }
			std::this_thread::sleep_for(std::chrono::milliseconds(20));    
		}
    });
}