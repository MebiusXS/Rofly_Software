/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 16:29:33
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 18:58:38
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>

class NetworkQualityPublisher
{
public:
    NetworkQualityPublisher() = delete;
    explicit NetworkQualityPublisher(ros::NodeHandle &nh);
    NetworkQualityPublisher(const NetworkQualityPublisher &rhs) = delete;
    NetworkQualityPublisher &operator=(const NetworkQualityPublisher &rhs) = delete;
    NetworkQualityPublisher(NetworkQualityPublisher &&rhs) = delete;
    NetworkQualityPublisher &operator=(NetworkQualityPublisher &&rhs) = delete;
    virtual ~NetworkQualityPublisher() = default;

    void publishNetworkQuality(const std::string &ip);

private:
    ros::NodeHandle nh_;
    ros::Publisher network_quality_pub_;

    std::string exec(const char *cmd);
    std::string getPingResult(const std::string &ip);
};
