/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-11 14:54:37
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-11 14:54:40
 */

#ifndef VIRTUAL_POS_PUBLISHER_H_
#define VIRTUAL_POS_PUBLISHER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

class VirtualPosPublisher
{
public:
    VirtualPosPublisher() = delete;
    VirtualPosPublisher(const ros::NodeHandle &nh);
    VirtualPosPublisher(const VirtualPosPublisher &rhs) = delete;
    VirtualPosPublisher &operator=(const VirtualPosPublisher &rhs) = delete;
    VirtualPosPublisher(VirtualPosPublisher &&rhs) = delete;
    VirtualPosPublisher &operator=(VirtualPosPublisher &&rhs) = delete;
    virtual ~VirtualPosPublisher() {}

    void set_pos(mavros_msgs::PositionTarget pose);
    void pos_publish();
    
    double normalize_yaw(double yaw);

private:
    ros::NodeHandle nh_;
    ros::Publisher pos_pub_;
    mavros_msgs::PositionTarget pos_msg_;
};

#endif // VIRTUAL_POS_PUBLISHER_H_