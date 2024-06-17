/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-03-29 10:55:34
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-03-29 10:55:39
 */

#include "virtual_rc/rc_control_publisher.h"

RCControlPublisher::RCControlPublisher(const ros::NodeHandle &nh) : nh_(nh)
{
    rc_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);
    rc_message_.channels = {1495, 1495, 1045, 1495, 1495, 1495, 1045}; // roll, pitch, throttle, yaw, mode, no_used, emergency_stop
}

void RCControlPublisher::rc_publish()
{
    rc_pub_.publish(rc_message_);
}

void RCControlPublisher::emergency_stop()
{
    rc_message_.channels[6] = 1945;
}

void RCControlPublisher::cancel_emergency_stop()
{
    rc_message_.channels[6] = 1045;
}

void RCControlPublisher::set_mode(RC_MODE mode)
{
    switch (mode)
    {
    case RC_MODE::STABILIZE:
        rc_message_.channels[4] = 1045;
        break;
    case RC_MODE::AIRGROUND:
        rc_message_.channels[4] = 1495;
        break;
    case RC_MODE::AUTOMATION:
        rc_message_.channels[4] = 1945;
        break;
    default:
        ROS_ERROR("INVALID RC MODE, SETTING TO AIRGROUND!");
        rc_message_.channels[4] = 1495;
        break;
    }
}

void RCControlPublisher::set_mode(RC_MODE_POS mode)
{
    switch (mode)
    {
    case RC_MODE_POS::STABILIZE:
        rc_message_.channels[4] = 1045;
        break;
    case RC_MODE_POS::POSITION:
        rc_message_.channels[4] = 1495;
        break;
    case RC_MODE_POS::AUTOMATION:
        rc_message_.channels[4] = 1945;
        break;
    default:
        ROS_ERROR("INVALID RC MODE, SETTING TO POSITION!");
        rc_message_.channels[4] = 1495;
        break;
    }
}

void RCControlPublisher::set_channels(int roll, int pitch, int throttle, int yaw, RC_MODE mode, bool emergency)
{
    rc_message_.channels[0] = roll;
    rc_message_.channels[1] = pitch;
    rc_message_.channels[2] = throttle;
    rc_message_.channels[3] = yaw;
    set_mode(mode);
    if (emergency)
    {
        emergency_stop();
    }
    else
    {
        cancel_emergency_stop();
    }
}

void RCControlPublisher::set_channels(int roll, int pitch, int throttle, int yaw, RC_MODE_POS mode, bool emergency)
{
    rc_message_.channels[0] = roll;
    rc_message_.channels[1] = pitch;
    rc_message_.channels[2] = throttle;
    rc_message_.channels[3] = yaw;
    set_mode(mode);
    if (emergency)
    {
        emergency_stop();
    }
    else
    {
        cancel_emergency_stop();
    }
}
