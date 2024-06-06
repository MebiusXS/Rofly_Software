/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 16:29:33
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 18:43:53
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>

class WifiQualityPublisher
{
public:
    WifiQualityPublisher() = delete;
    WifiQualityPublisher(ros::NodeHandle &nh, const std::string &wlan_name);
    WifiQualityPublisher(const WifiQualityPublisher &rhs) = delete;
    WifiQualityPublisher &operator=(const WifiQualityPublisher &rhs) = delete;
    WifiQualityPublisher(WifiQualityPublisher &&rhs) = delete;
    WifiQualityPublisher &operator=(WifiQualityPublisher &&rhs) = delete;
    virtual ~WifiQualityPublisher() = default;

    void publishNetworkQuality();

private:
    std::string wlan_name_;

    ros::NodeHandle nh_;
    ros::Publisher network_quality_pub_;

    std::string exec(const char *cmd);
    std::string getWifiSignalStrength();
};