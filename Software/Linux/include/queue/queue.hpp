#include <mutex>
#include <condition_variable>
#include <deque>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;

template<class T>
class BlockQueue {
public:
    using size_type = typename std::deque<T>::size_type;

public:
    BlockQueue(const int cap = -1) : m_maxCapacity(cap){}
    ~BlockQueue(){}

    BlockQueue(const BlockQueue &) = delete;
    BlockQueue &operator = (const BlockQueue &) = delete;

public:
    void put(const T t);
    T take();

    bool empty() const{
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }

    bool full() const{
        if(-1 == m_maxCapacity)
            return false;
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size() >= m_maxCapacity;
    }

    size_type size(){
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.size();
    }

public:
    bool offer(const T t);
    bool poll(T& t);
    bool offer(const T t, std::chrono::milliseconds& time);
    bool poll(T& t, std::chrono::milliseconds& time);

private:
    std::deque<T> m_queue;
    const int m_maxCapacity;
    mutable std::mutex m_mutex;
    std::condition_variable m_cond_empty;
    std::condition_variable m_cond_full;
};

template <class T>
void BlockQueue<T>::put(const T t){
    std::unique_lock<std::mutex> lock(m_mutex);
    if(m_maxCapacity != -1){
        m_cond_full.wait(lock, [this]{ return m_queue.size() < m_maxCapacity; });
    }
    m_queue.push_back(t);
    m_cond_empty.notify_all();
}

template <class T>
T BlockQueue<T>::take(){
    std::unique_lock<std::mutex> lock(m_mutex);
    // take必须判断队列为空
    m_cond_empty.wait(lock, [&](){return !m_queue.empty();});
    auto res = m_queue.front();
    m_queue.pop_front();
    m_cond_full.notify_all();
    return res;
}

template <class T>
bool BlockQueue<T>::offer(const T t){
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_maxCapacity != -1 && m_queue.size() >= m_maxCapacity){
        return false;
    }
    m_queue.push_back(t);
    m_cond_empty.notify_all();
    return true;
}

template <class T>
bool BlockQueue<T>::poll(T& t){
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_queue.empty()){
        return false;
    }
    t = m_queue.front();
    m_queue.pop_front();
    m_cond_full.notify_all();
    return true;
}

template <class T>
bool BlockQueue<T>::offer(const T t, std::chrono::milliseconds& time){
    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_maxCapacity != -1){
        bool result = m_cond_full.wait(lock, time, 
                                   [&]{ return m_queue.size() < m_maxCapacity; });
        if(!result){
            return false;
        }
    }
    m_queue.push_back(t);
    m_cond_empty.notify_all();
    return true;
}

template <class T>
bool BlockQueue<T>::poll(T& t, std::chrono::milliseconds& time){
    std::lock_guard<std::mutex> lock(m_mutex);
    bool result = m_cond_empty.wait_for(lock, time, 
                                   [&] {return !m_queue.empty();});
    if(!result){
        return false;
    }
    t = m_queue.front();
    m_queue.pop_front();
    m_cond_full.notify_all();
    return true;
}
