/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-03 16:49:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-15 14:33:50
 */

#include "ros/ros.h"
#include "socket/heartbeat_sender.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heartbeat_send_test");
    ros::NodeHandle nh, pnh("~");


    double loop_rate;
    std::string target_ip;
    if (!pnh.getParam("loop_rate", loop_rate) ||
        !pnh.getParam("target_ip", target_ip))
    {
        ROS_ERROR("[heartbeat_send] parameters not properly set");
        exit(1);
    }

    ros::Rate rate(loop_rate);

    HeartbeatSender sender(target_ip, 9999);

    while (ros::ok())
    {
        sender.sendHeartbeat("Heartbeat message from sender.");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}