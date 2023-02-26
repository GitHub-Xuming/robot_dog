#pragma once
#include "stdint.h"
#include "struct.h"

class intertialController
{
public:
    intertialController();
    ~intertialController();

    void RegistPosGetCallback(VirtualExcuterGetPosCallback callback);
    void PID_ControllerSetFbPos(float goal);
    float PID_Controller(float goal);
    void PID_ControllerParaInit(float first_pos);

private:
    float m_pid_goal_pos;
    float m_pid_kp;
    float m_pid_out_speed;
    float m_pid_bias;
    float m_pid_pre_pos;
    VirtualExcuterGetPosCallback m_cb_pos_get_;
};