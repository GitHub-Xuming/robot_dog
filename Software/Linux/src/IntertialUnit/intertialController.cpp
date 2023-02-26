#include "intertialController.h"


intertialController::intertialController()
{
    m_pid_kp = 10; //70
}

intertialController::~intertialController()
{

}

void intertialController::RegistPosGetCallback(VirtualExcuterGetPosCallback callback)
{
    m_cb_pos_get_ = callback;
}

void intertialController::PID_ControllerParaInit(float first_pos)
{
    //m_pid_pre_pos = first_pos; 
}

void intertialController::PID_ControllerSetFbPos(float goal)
{
    m_pid_pre_pos = goal;
}

float intertialController::PID_Controller(float goal)
{
	m_pid_goal_pos = goal;
    if(m_cb_pos_get_)
    {
	    //m_pid_pre_pos = m_cb_pos_get_();
        m_pid_bias = m_pid_goal_pos - m_pid_pre_pos;
        m_pid_out_speed = m_pid_kp * m_pid_bias;
    }

	return m_pid_out_speed;
}


