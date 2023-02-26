#pragma once
#include <string>
#include <iostream>
#include <semaphore.h>
#include <time.h>
#include <vector>


class ToolSem
{
public:
    ToolSem();
    ~ToolSem();
    inline timespec* GetAbsTime( size_t milliseconds, timespec& absTime );
    bool SemTimedWait( size_t timeout );
    void SemPost();
private:
    sem_t m_sem_;
};