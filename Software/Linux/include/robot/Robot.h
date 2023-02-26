#pragma once
#include <memory>
#include <thread>
#include <vector>
#include <map>
#include <semaphore.h>
#include "struct.h"
#include "filter.h"
#include "ArmCtrlBase.h"
#include "DogCtrlBase.h"
#include "sem.h"
#include "intertialController.h"
#include "virtrualExecuter.h"
#include "remote.h"

enum AnalogChannelOn
{
    ChannelOnDog,
    ChannelOnArm,
};

struct AxisCtrlQuantity
{
    float move_x;
    float move_y;
    float move_z;
    float rotate_x;
    float rotate_y;
    float rotate_z;
    float axis_claw;
};

class Robot
{
public:
    Robot();
    ~Robot();
    void RegistRobotArmImpl(std::shared_ptr<ArmCtrlBase> arm_ptr);
    void RegistRobotDogImpl(std::shared_ptr<DogCtrlBase> dog_ptr);

private:
    void ParaInit();
    bool RobotDataReq(DataReqModel &req);
    void IMU_PosReq();
    void IMU_PosFB(coord_point &pos);
    void ArmDisable();
    void ArmAngleFB(ArmAngleFeedBack &angle);
    void ArmStateMachine();
    void RobotDogControl();
    void ConstructAndSendFrame(uint8_t function_code, uint8_t data_len, std::vector<uint8_t> data);
    void robotRun();
    void ArmCtrlInit(); 
    void ArmCtrl();
    void ArmStretch();
    void ArmContract();
    float AngleConvert(ArmAxis &axis, float in);
    void EnableJudgment();
    void AnalogChannelMapping();
private:
    std::shared_ptr<ArmCtrlBase> m_arm_ctr_ = nullptr;
    std::shared_ptr<DogCtrlBase> m_dog_ctr_ = nullptr;

    std::shared_ptr<intertialController> m_pid_controller_ = nullptr;
    std::shared_ptr<virtralExecuter> m_vir_executer_ = nullptr;
    double m_light_first_val_;
    DataReqModel m_angle_req_;
    DataReqModel m_imu_pos_req_;
    Switch m_switch_ch_;
    AnalogChannelOn m_analog_ch_map_ = AnalogChannelOn::ChannelOnDog;

	float m_arm_kalman_Q_ = 0.1;
	float m_arm_kalman_R_ = 1.0;
    Filter  m_arm_filter_mx_;
	Filter  m_arm_filter_my_;
	Filter  m_arm_filter_mz_;
	Filter  m_arm_filter_rx_;
	Filter  m_arm_filter_ry_;
	Filter  m_arm_filter_rz_;

	float m_dog_kalman_Q_ = 0.5;
	float m_dog_kalman_R_ = 1.0;
    Filter  m_dog_filter_mx_;
	Filter  m_dog_filter_my_;
	Filter  m_dog_filter_mz_;
	Filter  m_dog_filter_rx_;
	Filter  m_dog_filter_ry_;
	Filter  m_dog_filter_rz_;

    Filter m_imu_ctrl_;

	std::thread m_thread_;
	bool m_thread_run_;
    Pose m_dog_posture_;
    Pose m_arm_point_;
    coord_point m_imu_pos_;
    FootCoord m_foot_pos_;
    ArmAngleFeedBack m_arm_first_angle_;

    bool m_arm_res_flg_ = false;
    bool m_is_enable_ = false;
    bool m_movez_alarm_flg_ = true;
    SERIAL_SEND m_send_data_;
    struct timespec m_time_ = {0, 0};

    AxisCtrlQuantity m_axis_ctrl_quantity_;
    std::vector<ArmAxis> m_arm_axis_;
    std::vector<uint8_t> m_servo_data_;
    sem_t m_sem_;
};