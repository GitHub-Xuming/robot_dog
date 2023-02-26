#pragma once
#include <stdint.h>
#include <vector>
#include <array>
#include <math.h>
#include <string>
#include <list>
#include <thread>
#include <mutex>
#include "struct.h"
#include "DataProgress.h"
#include "singleton.hpp"
#include "filter.h"

using namespace std;

#define         ABS(a)                  ((a) > 0 ? (a) : (-a))
#define         KEY_RECOG_LEN           32

class Interpolater;

enum SwitchStatus
{
    off,
    middle,
    on,
};

enum KeyStatus
{
    idle,
    short_press,
    long_press,
    double_click,
};

struct Switch
{
    SwitchStatus switch_A;
    SwitchStatus switch_B;
    SwitchStatus switch_C;
    SwitchStatus switch_D;
};

struct Key
{
    KeyStatus key_S;
    KeyStatus Last_key_S;
};

struct InterpolateParam
{
    int64_t in; 
    int64_t x0;
    int64_t x1;

    float y0;
    float y1;
    float out;
    float out_last;
};

/*线性插值器*/
class Interpolater : public Singleton<Interpolater>
{
public:
    Interpolater();
    ~Interpolater(){};

    void InterpolateCompute(InterpolateParam *ptr);
    
private:
    InterpolateParam *m_param_ptr_;

};

struct AnalogChannel
{
    float value_out;
    float value_last;
    float value_pre;
    bool first_flg;
    InterpolateParam interpolate_para;
};

struct AllAnalogChannel
{
    AnalogChannel ch_1;
    AnalogChannel ch_2;
    AnalogChannel ch_3;
    AnalogChannel ch_4;
    AnalogChannel ch_9;
    AnalogChannel ch_10;
    uint64_t TimeStampMs;
};

struct KeyRecognition
{
    Key key_;
    std::list<int8_t> key_list;
    std::array<int8_t, KEY_RECOG_LEN> key_pattern;
    bool Long_press_flg;
};


class RemoteController : public Singleton<RemoteController>
{
public:
    RemoteController();
    ~RemoteController();

    void RobotRemotFB(RobotRemotFeedBack &channel_val);
    float ChannelPreProcess(float in, float threshold);
    void RemoteInterpolation();
    void AnalogChannelProcess();
    void SwitchChannelProcess();
    void KeyStatusProcess();
    void GetLongPressStatus(bool &status);
    void GetSwitchStatus(Switch &sw);
    void GetAnalogChannel(AllAnalogChannel *ac);
    void ClearKeyStatus();
    void RemoteProtected(AnalogChannel &obj);
private:
    std::array<uint16_t, 10> m_channel_val_;
    std::mutex m_lock_;
    KeyRecognition m_key_L_;
    Switch m_switch_;
    AllAnalogChannel m_analog_ch_Interpolate_;
    //*本次与上次用于轮询计算*/
    AllAnalogChannel m_analog_ch_;
    AllAnalogChannel m_analog_ch_last_;
    std::array<Filter, 10> m_channel_filter_;
	float m_kalman_Q_ = 0.1;
	float m_kalman_R_ = 1.0;
};
