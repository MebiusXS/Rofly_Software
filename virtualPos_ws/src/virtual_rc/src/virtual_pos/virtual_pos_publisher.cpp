/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-11 14:54:37
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-11 17:33:25
 */

#include "virtual_pos/virtual_pos_publisher.h"

VirtualPosPublisher::VirtualPosPublisher(const ros::NodeHandle &nh) : nh_(nh)
{
    pos_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
}

void VirtualPosPublisher::pos_publish()
{
    pos_pub_.publish(pos_msg_);
}

void VirtualPosPublisher::set_pos(mavros_msgs::PositionTarget pose)
{
    pos_msg_ = pose;
}

double VirtualPosPublisher::normalize_yaw(double yaw)
{
    while (yaw > M_PI) yaw -= 2 * M_PI;
    while (yaw < -M_PI) yaw += 2 * M_PI;
    return yaw;
}