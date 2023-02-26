#include "sem.h"

ToolSem::ToolSem()
{

}

ToolSem::~ToolSem()
{
    
}

// 获取需要延时等待时间的绝对时间戳
inline timespec* ToolSem::GetAbsTime( size_t milliseconds, timespec& absTime )
{
	// CLOCK_REALTIME：系统实时时间,随系统实时时间改变而改变,即从UTC1970-1-1 0:0:0开始计时,
	//                 中间时刻如果系统时间被用户改成其他,则对应的时间相应改变
    clock_gettime( CLOCK_REALTIME, &absTime );
    
	absTime.tv_sec += milliseconds / 1000;
    absTime.tv_nsec += (milliseconds % 1000) * 1000000;

    // 纳秒进位秒
    if( absTime.tv_nsec >= 1000000000 )
    {
        absTime.tv_sec += 1;
        absTime.tv_nsec -= 1000000000;
    }

   return &absTime;
}

// sem_timedwait 实现的睡眠 -- 存在缺陷
// 如果信号量大于0，则减少信号量并立马返回true
// 如果信号量小于0，则阻塞等待，当阻塞超时时返回false
bool ToolSem::SemTimedWait( size_t timeout )
{
    timespec absTime;
    // 获取需要延时等待时间的绝对时间戳
    GetAbsTime( timeout, absTime );
    if( sem_timedwait( &m_sem_, &absTime ) != 0 )
    {
        return false;
    }
    return true;
}

void ToolSem::SemPost()
{
    sem_post(&m_sem_);
}