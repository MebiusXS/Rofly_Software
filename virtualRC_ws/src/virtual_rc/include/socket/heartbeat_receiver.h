/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-03 17:46:29
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-03 22:02:10
 */

#include <iostream>

#include "ros/ros.h"

#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
class HeartbeatReceiver
{
public:
    HeartbeatReceiver() = delete;
    HeartbeatReceiver(const int port, const double land_timeout, const double hover_timeout);
    HeartbeatReceiver(const HeartbeatReceiver &rhs) = delete;
    HeartbeatReceiver &operator=(const HeartbeatReceiver &rhs) = delete;
    HeartbeatReceiver(HeartbeatReceiver &&rhs) = delete;
    HeartbeatReceiver &operator=(HeartbeatReceiver &&rhs) = delete;
    virtual ~HeartbeatReceiver();

    bool shouldLand();
    bool shouldHover();
    void listenForHeartbeats();

private:
    int n_;
    int port_;
    int sockfd_;
    char buffer_[1024];

    double land_timeout_;
    double hover_timeout_;

    socklen_t len_;
    struct sockaddr_in cliaddr_;
    struct sockaddr_in servaddr_;

    ros::Time last_heartbeat_time_;
};