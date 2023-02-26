#include "socket_paint.h"

SocketPaint::SocketPaint()
{

}

SocketPaint::~SocketPaint()
{

}

void SocketPaint::Init()
{
    signal(SIGPIPE, SIG_IGN);
    m_address_.sin_family = AF_INET;
    inet_pton(AF_INET, SERVER_ADDR, &m_address_.sin_addr.s_addr);
    m_address_.sin_port = htons(SERVER_PORT);
    m_sockFd_ = socket(AF_INET, SOCK_STREAM, 0);

    cout << "connect to server ..." << endl;
    int ret = connect(m_sockFd_, (struct sockaddr*)&m_address_, sizeof(m_address_));
    
    if(ret == -1)
    {
        printf("connected failed\n");
    }
}

void SocketPaint::ConstructSendFrame(uint8_t function_code, uint8_t data_len, std::vector<uint8_t> data)
{
    uint8_t check_sum = 0;

    m_send_data_.head = PAINT_HEAD;
    m_send_data_.function_code = function_code;
    m_send_data_.data_len = data_len;
    m_send_data_.data = data;
 
    check_sum += m_send_data_.function_code;
    check_sum += m_send_data_.data_len;
    for(int i = 0; i < m_send_data_.data.size(); i++)
    {
        check_sum += m_send_data_.data[i];
    }
    m_send_data_.check_sum = check_sum;

    m_send_buff_[0] = m_send_data_.head;
    m_send_buff_[1] = m_send_data_.function_code;
    m_send_buff_[2] = m_send_data_.data_len;
    for(int i = 0; i < m_send_data_.data.size(); i++)
    {
        m_send_buff_[3 + i] = m_send_data_.data[i];
    }
    m_send_buff_[3 + m_send_data_.data.size()] = m_send_data_.check_sum;

    int len = m_send_data_.data.size() + 4;

    int ret = send(m_sockFd_, m_send_buff_, len, 0);
    if(ret < 0)
    {
        //printf("client send failed\n");
    }
}

void SocketPaint::DrawPoint(uint8_t line, uint8_t hue, int16_t value)
{
    uint8_t function_code = 0;
    uint8_t data_len = 0;
    std::vector<uint8_t> data;
    data.resize(4);
    
    function_code = RCV_TYPE_XY_POINT;
    data_len = 4;
    data[0] = line;
    data[1] = hue;
    data[2] = value & 0xff;
    data[3] = (value >> 8) & 0xff;
    
    ConstructSendFrame(function_code, data_len, data);
}


