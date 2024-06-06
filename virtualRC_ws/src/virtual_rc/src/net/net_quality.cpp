/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 16:29:33
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 18:58:49
 */

#include <net/net_quality.h>

NetworkQualityPublisher::NetworkQualityPublisher(ros::NodeHandle &nh) : nh_(nh)
{
    network_quality_pub_ = nh_.advertise<std_msgs::String>("network_quality", 10);
}

void NetworkQualityPublisher::publishNetworkQuality(const std::string& ip)
{
    std_msgs::String msg;
    msg.data = getPingResult(ip);

    network_quality_pub_.publish(msg);
    ROS_INFO("Network Quality to %s: %s", ip.c_str(), msg.data.c_str());
}

std::string NetworkQualityPublisher::exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

std::string NetworkQualityPublisher::getPingResult(const std::string& ip)
{
    std::string command = "ping -c 4 " + ip + " | tail -1 | awk '{print $4}' | cut -d '/' -f 2";
    std::string output = exec(command.c_str());
    return "Avg Delay: " + output + " ms";
}