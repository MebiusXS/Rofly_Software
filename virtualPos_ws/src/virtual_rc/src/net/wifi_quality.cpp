/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 16:29:33
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 18:55:50
 */

#include <net/wifi_quality.h>

WifiQualityPublisher::WifiQualityPublisher(ros::NodeHandle &nh, const std::string &wlan_name) : nh_(nh), wlan_name_(wlan_name)
{
    network_quality_pub_ = nh_.advertise<std_msgs::String>("wifi_signal_strength", 10);
}

void WifiQualityPublisher::publishNetworkQuality()
{
    std_msgs::String msg;
    msg.data = getWifiSignalStrength();

    network_quality_pub_.publish(msg);
    ROS_INFO("WiFi Signal Strength: %s", msg.data.c_str());
}

std::string WifiQualityPublisher::exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }

    return result;
}

std::string WifiQualityPublisher::getWifiSignalStrength()
{
    std::string command = "iw dev " + wlan_name_ + " link | grep 'signal:' | awk '{print $2}'";
    std::string output = exec(command.c_str());
    return output;
}