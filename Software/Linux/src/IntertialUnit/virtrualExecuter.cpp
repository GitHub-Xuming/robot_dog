#include "virtrualExecuter.h"

virtralExecuter::virtralExecuter()
{
}

virtralExecuter::~virtralExecuter()
{
}

void virtralExecuter::virtralExecuterParaInit(float first_pos)
{
    m_exc_pre_pos = first_pos;
}

void virtralExecuter::virtralExecuterSetGoalSpeed(float goal)
{
	m_exc_run_speed = goal;
}

void virtralExecuter::VirtualExcuterCompute()
{
	m_exc_pre_pos += m_exc_run_speed * 0.02;
}

float virtralExecuter::VirtualExcuterGetPos()
{
	return m_exc_pre_pos;
}




