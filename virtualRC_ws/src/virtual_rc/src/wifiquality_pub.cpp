/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 18:31:31
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 18:56:05
 */

#include <net/wifi_quality.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifiquality_pub");
    ros::NodeHandle nh, pnh("~");
    ros::Rate rate(10);

    std::string wlan_name;

    if (!pnh.getParam("wlan_name", wlan_name))
    {
        ROS_ERROR("[wifi_quality] parameters not properly set");
        exit(1);
    }

    WifiQualityPublisher publisher(nh, wlan_name);

    while (ros::ok())
    {
        publisher.publishNetworkQuality();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}