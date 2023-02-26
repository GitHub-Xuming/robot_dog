#pragma once
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <vector>
#include <stdint.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include "singleton.hpp"

#define     SERVER_PORT                 8000
#define     RCV_TYPE_XY_POINT           0x51
#define     SERVER_ADDR                 "192.168.1.12"
#define     PAINT_HEAD                  0x55

using namespace std;

typedef struct
{
	uint8_t head;
    uint8_t function_code;
    uint8_t data_len;
    uint8_t check_sum;
    std::vector<uint8_t> data;
}SocketSend;

class SocketPaint : public Singleton<SocketPaint>
{
public:
    SocketPaint();
    ~SocketPaint();
    void Init();
    void DrawPoint(uint8_t line, uint8_t hue, int16_t value);

private:
    void ConstructSendFrame(uint8_t function_code, uint8_t data_len, std::vector<uint8_t> data);

private:
    int m_sockFd_ = 0;
    char m_buffer_[1024] = { 0 };
    struct sockaddr_in m_address_;
    SocketSend m_send_data_;
    uint8_t m_send_buff_[100] = { 0 };
};
