/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-03 16:49:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-03 16:49:38
 */

#include "socket/heartbeat_sender.h"

HeartbeatSender::HeartbeatSender(const std::string &ip, int port) : ip_(ip), port_(port)
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0)
    {
        std::cerr << "Socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port_);
    servaddr_.sin_addr.s_addr = inet_addr(ip_.c_str());
}

HeartbeatSender::~HeartbeatSender()
{
    close(sockfd_);
}

void HeartbeatSender::sendHeartbeat(const std::string &message)
{
    sendto(sockfd_, message.c_str(), message.length(),
           MSG_CONFIRM, (const struct sockaddr *)&servaddr_, sizeof(servaddr_));
    std::cout << "[" << ros::Time::now() << "] "  << ip_ << " Heartbeat message sent." << std::endl;
}