#include <iostream>
#include <stdlib.h>
#include "my_debug.h"
#include "DataProgress.h"

DataProgress::DataProgress()
{
    m_is_run_ = true;
    SerialInit();
}

DataProgress::~DataProgress()
{
    m_is_run_ = false;
}

void DataProgress::RegistArmAngleCallback(ArmAngleCallback callback)
{
    m_cb_arm_angle_ = callback;
}

void DataProgress::RegistIMU_PosCallback(IMU_PosCallback callback)
{
    m_cb_imu_pos_ = callback;
}

void DataProgress::RegistRobotDogRemotCallback(RobotDogRemotCallback callback)
{
    m_cb_robot_remot_ = callback;
}

void DataProgress::SerialInit()
{
    m_send_queue_ = std::make_shared<BlockQueue<SERIAL_SEND>>(10);
    m_serial_connect_ = std::make_shared<CSerialConnection>();
    m_serial_connect_->setBaud(1000000);
    m_serial_connect_->setPort(m_com_path_.c_str());
    if(m_serial_connect_->openSimple())
    {
        m_is_open_ = true;
        printf("[AuxCtrl] Open serail port_test sucessful!\n");
        printf("baud rate:%d\n",m_serial_connect_->getBaud());
    }
    else
    {
        printf("[AuxCtrl] Open serail port test %s failed! \n", m_com_path_.c_str());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    SerialProcess();

	m_send_thread_ = std::thread([this]
	{
		SERIAL_SEND send_data;
		while(m_is_run_)
		{
			send_data = m_send_queue_->take();
			Send(send_data);
            std::this_thread::sleep_for(std::chrono::microseconds(300)); 
		}
	});
}

void DataProgress::SerialSend(SERIAL_SEND &send_data)
{
    m_send_queue_->put(send_data);
}

void DataProgress::SerialProcess()
{
    m_th_process_ = std::thread([this]
    {
        char ch, buff[10] = { 0 };
        int count = 0;
        while(m_is_run_)
        {
            if(m_is_open_)
            {
                int res = m_serial_connect_->read((char *)&ch, 1, 100);
                if(res == 1)
                {
                    USART_Receiver(ch);         
                }
            }
            else
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

void DataProgress::DataDispense(std::vector<char> data)
{
    switch(data[0])
    {
        case RCV_TYPE_REMOTE_CTRL:
        m_robot_remot_fb_.assign(data.begin() + 2, data.end());   
        if(m_cb_robot_remot_) 
        {
            m_cb_robot_remot_(m_robot_remot_fb_);
        }
        break;

        case TYPE_ARM_POS_RES:
        m_arm_angle_fb_.axis_angle.axis_1 = ((data[2] << 8) | data[3]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.axis_angle.axis_2 = ((data[4] << 8) | data[5]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.axis_angle.axis_3 = ((data[6] << 8) | data[7]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.axis_angle.axis_4 = ((data[8] << 8) | data[9]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.axis_angle.axis_5 = ((data[10] << 8) | data[11]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.axis_angle.axis_6 = ((data[12] << 8) | data[13]) / ANGLE_CONVERT_RATE;
        m_arm_angle_fb_.claw_angle = ((data[14] << 8) | data[15]) / ANGLE_CONVERT_RATE;
        m_cb_arm_angle_(m_arm_angle_fb_);
        break;

        case TYPE_IMU_POS_RES:
        m_imu_pos_fb_.x = ((int16_t)((data[2] << 8) | data[3])) / 10.0;
        m_imu_pos_fb_.y = ((int16_t)((data[4] << 8) | data[5])) / 10.0;
        m_imu_pos_fb_.z = ((int16_t)((data[6] << 8) | data[7])) / 10.0;
        m_cb_imu_pos_(m_imu_pos_fb_);
        break;

        default:  break;
    }	
}

void DataProgress::USART_Receiver(char in_data)
{
    m_recv_buf_[m_counter_] = in_data;   

    if(m_counter_ == 0 && m_recv_buf_[0] != 0x55) 
    {
        printf("progress break!\n");
        return;      //第 0 号数据不是帧头，跳过
    }
    m_counter_++; 
    if(m_counter_ == 3)
    {
        m_receive_len_ = m_recv_buf_[2];
    }
    if(m_counter_ == (m_receive_len_ + 4)) //head func len sum
    { 
        for(int i = 1; i < m_counter_ - 1; i++)
        {
            m_sum_ += m_recv_buf_[i];  
        }  
        if((m_recv_buf_[m_counter_ - 1] == m_sum_) && (m_recv_buf_[0] == 0x55))
        {
            std::vector<char> tmp(m_recv_buf_.begin() + 1, m_recv_buf_.begin() + 3 + m_receive_len_);
            DataDispense(tmp);
        }
        else
        {
            printf("progress data error!\n");
        }
        m_counter_ = 0; 
        m_sum_ = 0;
    }
} 

void DataProgress::Send(SERIAL_SEND &send)
{
    int len = 0;
    if(m_is_open_)
    {
        m_send_buff_[0] = (char)send.head;
        m_send_buff_[1] = (char)send.function_code;
        m_send_buff_[2] = (char)send.data_len;
        for(int i = 0; i < send.data.size(); i++)
        {
            m_send_buff_[3 + i] = (char)send.data[i];
        }
        m_send_buff_[3 + send.data.size()] = (char)send.check_sum;
        len = send.data.size() + 4;

        m_serial_connect_->write(m_send_buff_, len);
    }
}


