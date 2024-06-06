/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-03 16:49:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-03 16:49:38
 */

#include <iostream>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <ros/ros.h>

class HeartbeatSender
{
public:
    HeartbeatSender() = delete;
    HeartbeatSender(const std::string &ip, int port);
    HeartbeatSender(const HeartbeatSender &rhs) = delete;
    HeartbeatSender &operator=(const HeartbeatSender &rhs) = delete;
    HeartbeatSender(HeartbeatSender &&rhs) = delete;
    HeartbeatSender &operator=(HeartbeatSender &&rhs) = delete;
    virtual ~HeartbeatSender();

    void sendHeartbeat(const std::string &message);

private:
    int port_;
    int sockfd_;
    std::string ip_;
    struct sockaddr_in servaddr_;
};