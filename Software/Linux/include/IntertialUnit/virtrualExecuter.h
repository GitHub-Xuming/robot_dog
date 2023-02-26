#pragma once
#include "stdint.h"

class virtralExecuter
{
public:
    virtralExecuter();
    ~virtralExecuter();

    void virtralExecuterParaInit(float first_pos);
    void virtralExecuterSetGoalSpeed(float goal);
    void VirtualExcuterCompute();
    float VirtualExcuterGetPos();

private:
    float m_exc_pre_pos = 0;
    float m_exc_run_speed = 0;
};