/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-07 17:53:17
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-07 17:53:20
 */

#include <net/net_quality.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "netquality_pub");
    ros::NodeHandle nh;
    ros::Rate rate(100);

    std::string target_ip = "192.168.51.142"; 

    NetworkQualityPublisher publisher(nh);

    while (ros::ok())
    {
        publisher.publishNetworkQuality(target_ip);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}