
#ifndef __DATA_PROGRESS_H__
#define __DATA_PROGRESS_H__

#include <stdint.h>
#include <vector>
#include <math.h>
#include <string>
#include <thread>
#include "queue.hpp"
#include "struct.h"
#include "CSerialConnection.h"
#include "singleton.hpp"

using namespace std;
using namespace everest::hwdrivers;


class DataProgress : public Singleton<DataProgress>
{
public:
    DataProgress();
    ~DataProgress();

    void SerialInit();
    void SerialProcess();
    void Send(SERIAL_SEND &send);
    void USART_Receiver(char in_data);
    void DataDispense(std::vector<char> data);
    void SerialSend(SERIAL_SEND &send_data);
    void RegistArmAngleCallback(ArmAngleCallback callback);
    void RegistIMU_PosCallback(IMU_PosCallback callback);
    void RegistRobotDogRemotCallback(RobotDogRemotCallback callback);
private:
    std::shared_ptr<CSerialConnection> m_serial_connect_;
    std::thread m_th_process_;
    char m_send_buff_[50];
    bool m_is_open_ = false;
    bool m_is_run_ = false;
    string m_com_path_ = "/dev/ttyO2";

    ArmAngleFeedBack m_arm_angle_fb_;
    RobotRemotFeedBack m_robot_remot_fb_;
    coord_point m_imu_pos_fb_;
    ArmAngleCallback m_cb_arm_angle_; //机械臂初始角度请求回调
    IMU_PosCallback m_cb_imu_pos_; //imu位置请求回调
    RobotDogRemotCallback m_cb_robot_remot_;

    std::vector<char> m_recv_buf_ = std::vector<char>(RECEIVE_MAX_NUM, 0);
    std::shared_ptr<BlockQueue<SERIAL_SEND>> m_send_queue_;
    std::thread m_send_thread_;
    uint8_t m_sum_ = 0;
    int m_counter_ = 0;
    int m_receive_len_ = 0;
};




#endif 




