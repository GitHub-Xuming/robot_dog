#include "remote.h"
#include "my_debug.h"
#include <time.h>
#include<chrono>
#include "socket_paint.h"

Interpolater::Interpolater()
{
}

void Interpolater::InterpolateCompute(InterpolateParam *ptr)
{
    double delta = 0.0;
    //printf("------Interpolate param,x0:%d,y0:%f,x1:%d,y1:%f\n", ptr->x0,ptr->y0,ptr->x1,ptr->y1);
    ptr->out = ptr->y0 + (ptr->in - ptr->x0) * (ptr->y1 - ptr->y0) / (ptr->x1 - ptr->x0);
    delta = fabs(ptr->out_last - ptr->out);
    ptr->out_last = ptr->out;
}

RemoteController::RemoteController()
{
    m_channel_val_.fill(0);
    DataProgress::Instance().RegistRobotDogRemotCallback(
        std::bind(&RemoteController::RobotRemotFB, this, std::placeholders::_1));

    m_analog_ch_.ch_1.first_flg = true;
    m_analog_ch_.ch_2.first_flg = true;
    m_analog_ch_.ch_3.first_flg = true;
    m_analog_ch_.ch_4.first_flg = true;
    m_analog_ch_.ch_9.first_flg = true;
    m_analog_ch_.ch_10.first_flg = true;

    for(int i = 0; i < 10; i++)
    {
        m_channel_filter_[i].KalmanFilterInit(500, m_kalman_Q_, m_kalman_R_);
    }

    m_key_L_.key_.Last_key_S = KeyStatus::idle;
    m_key_L_.key_pattern = {-1, -1, -1, -1, -1, -1, -1, -1,
                            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                           -1, -1, -1, -1, -1, -1, -1, -1};
}

RemoteController::~RemoteController()
{
}

void RemoteController::RobotRemotFB(RobotRemotFeedBack &channel_val)
{
    uint16_t temp = 0;
    std::unique_lock<std::mutex> lock(m_lock_);

    for(int i = 0; i < 4; i++)
    {
        temp = ((channel_val[i * 2] << 8) | channel_val[i * 2 + 1]);
        //m_channel_val_[i] = temp;
        m_channel_val_[i] = m_channel_filter_[i].KalmanFilterFun((float)temp);
    }
    m_channel_val_[4] = ((channel_val[8] << 8) | channel_val[9]);
    m_channel_val_[5] = ((channel_val[10] << 8) | channel_val[11]);
    m_channel_val_[6] = ((channel_val[12] << 8) | channel_val[13]);
    m_channel_val_[7] = ((channel_val[14] << 8) | channel_val[15]);

    temp = ((channel_val[16] << 8) | channel_val[17]);
    m_channel_val_[8] = m_channel_filter_[8].KalmanFilterFun((float)temp);
    temp = ((channel_val[18] << 8) | channel_val[19]);
    m_channel_val_[9] = m_channel_filter_[9].KalmanFilterFun((float)temp);

    KeyStatusProcess(); //10HZ被调用，取决于单片机上报频率
    AnalogChannelProcess();
    SwitchChannelProcess();
}

float RemoteController::ChannelPreProcess(float in, float threshold)
{
    float out = 0.0;

    if((in > (500 + threshold)) || (in < (500 - threshold)))
    {
        if(in > (500 + threshold))
            out = in - (500 + threshold);
        if(in < (500 - threshold))
            out = in - (500 - threshold);
    }
    else
    {
        out = 0;
    }
    
    return out;
}

void RemoteController::RemoteProtected(AnalogChannel &obj)
{
    float delta = fabs(obj.value_pre - obj.value_last);
    //printf("robot remot delta:%f\n", delta); 
    if(!obj.first_flg)
    {
        if(delta > 400.0)
        {
            printf("robot remot obj delta:%f\n", delta);
        }
        obj.value_out = obj.value_pre;
        obj.value_last = obj.value_pre;
    }
    else
    {
        obj.value_last = obj.value_pre;
        obj.first_flg = false;
    }
}

void RemoteController::AnalogChannelProcess()
{
    m_analog_ch_last_ = m_analog_ch_; 

    m_analog_ch_.ch_1.value_pre = (float)m_channel_val_[0];
    m_analog_ch_.ch_2.value_pre = (float)m_channel_val_[1];
    m_analog_ch_.ch_3.value_pre = (float)m_channel_val_[2];
    m_analog_ch_.ch_4.value_pre = (float)m_channel_val_[3];
    m_analog_ch_.ch_9.value_pre = (float)m_channel_val_[8];
    m_analog_ch_.ch_10.value_pre = (float)m_channel_val_[9];

    RemoteProtected(m_analog_ch_.ch_1);
    RemoteProtected(m_analog_ch_.ch_2);
    RemoteProtected(m_analog_ch_.ch_3);
    RemoteProtected(m_analog_ch_.ch_4);
    RemoteProtected(m_analog_ch_.ch_9);
    RemoteProtected(m_analog_ch_.ch_10);

    /*记录当前时间戳*/
    auto timeNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
    m_analog_ch_.TimeStampMs = timeNow.count();
}

void RemoteController::RemoteInterpolation()
{
    uint64_t time_now = 0;
    auto timeNow = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

    time_now = timeNow.count() - 100;
    /*通道1 插值*/
    m_analog_ch_Interpolate_.ch_1.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_1.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_1.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_1.interpolate_para.y0 = m_analog_ch_last_.ch_1.value_out;
    m_analog_ch_Interpolate_.ch_1.interpolate_para.y1 = m_analog_ch_.ch_1.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_1.interpolate_para));

    /*通道2 插值*/
    m_analog_ch_Interpolate_.ch_2.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_2.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_2.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_2.interpolate_para.y0 = m_analog_ch_last_.ch_2.value_out;
    m_analog_ch_Interpolate_.ch_2.interpolate_para.y1 = m_analog_ch_.ch_2.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_2.interpolate_para));

    /*通道3 插值*/
    m_analog_ch_Interpolate_.ch_3.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_3.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_3.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_3.interpolate_para.y0 = m_analog_ch_last_.ch_3.value_out;
    m_analog_ch_Interpolate_.ch_3.interpolate_para.y1 = m_analog_ch_.ch_3.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_3.interpolate_para));

    /*通道4 插值*/
    m_analog_ch_Interpolate_.ch_4.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_4.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_4.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_4.interpolate_para.y0 = m_analog_ch_last_.ch_4.value_out;
    m_analog_ch_Interpolate_.ch_4.interpolate_para.y1 = m_analog_ch_.ch_4.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_4.interpolate_para));

    /*通道9 插值*/
    m_analog_ch_Interpolate_.ch_9.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_9.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_9.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_9.interpolate_para.y0 = m_analog_ch_last_.ch_9.value_out;
    m_analog_ch_Interpolate_.ch_9.interpolate_para.y1 = m_analog_ch_.ch_9.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_9.interpolate_para));

    /*通道10 插值*/
    m_analog_ch_Interpolate_.ch_10.interpolate_para.in = time_now;
    m_analog_ch_Interpolate_.ch_10.interpolate_para.x0 = m_analog_ch_last_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_10.interpolate_para.x1 = m_analog_ch_.TimeStampMs;
    m_analog_ch_Interpolate_.ch_10.interpolate_para.y0 = m_analog_ch_last_.ch_10.value_out;
    m_analog_ch_Interpolate_.ch_10.interpolate_para.y1 = m_analog_ch_.ch_10.value_out;
    Interpolater::Instance().InterpolateCompute(&(m_analog_ch_Interpolate_.ch_10.interpolate_para));
}

void RemoteController::SwitchChannelProcess()
{
    if(m_channel_val_[4] > 200)
    {
        m_switch_.switch_D = SwitchStatus::on;
    }
    else
    {
        m_switch_.switch_D = SwitchStatus::off;
    }
    if(m_channel_val_[5] > 200)
    {
        m_switch_.switch_C = SwitchStatus::on;
    }
    else
    {
        m_switch_.switch_C = SwitchStatus::off;
    }
    if(m_channel_val_[6] > 200)
    {
        m_switch_.switch_B = SwitchStatus::on;
    }
    else
    {
        m_switch_.switch_B = SwitchStatus::off;
    }
}

void RemoteController::KeyStatusProcess()
{
    int8_t tmp = 0;
    KeyStatus tmp_key = KeyStatus::idle;
    int sum = 0;

    m_channel_val_[7] > 200 ? (tmp = 1) : (tmp = -1);
    m_key_L_.key_list.push_back(tmp);
    if(m_key_L_.key_list.size() > KEY_RECOG_LEN)
    {
        m_key_L_.key_list.pop_front();
        auto it = m_key_L_.key_list.begin();
        for(int i = 0; i < KEY_RECOG_LEN; i++)
        {
            sum += (*it * m_key_L_.key_pattern[i]);
            it++;
        }
    }

    if(sum > 20)
    {
        tmp_key = KeyStatus::long_press;
    }
    else
    {
        tmp_key = KeyStatus::idle;
        m_key_L_.Long_press_flg = false;  
    }
    
    m_key_L_.key_.key_S = tmp_key;
    if(m_key_L_.key_.Last_key_S != m_key_L_.key_.key_S)  
    {
        if(m_key_L_.key_.key_S == KeyStatus::long_press) 
        {
            m_key_L_.Long_press_flg = true; 
            printf("detect long press!\n");
        }
    }
    m_key_L_.key_.Last_key_S = m_key_L_.key_.key_S;
}

void RemoteController::ClearKeyStatus()
{
    std::unique_lock<std::mutex> lock(m_lock_);  
    printf("clear key status\n"); 
    m_key_L_.Long_press_flg = false;
}

void RemoteController::GetLongPressStatus(bool &status)
{
    std::unique_lock<std::mutex> lock(m_lock_);
    status = m_key_L_.Long_press_flg;
}

void RemoteController::GetSwitchStatus(Switch &sw)
{
    std::unique_lock<std::mutex> lock(m_lock_);
    sw = m_switch_;
}

void RemoteController::GetAnalogChannel(AllAnalogChannel *ac)
{
    std::unique_lock<std::mutex> lock(m_lock_);
    RemoteInterpolation();
    *ac = m_analog_ch_Interpolate_;
}
