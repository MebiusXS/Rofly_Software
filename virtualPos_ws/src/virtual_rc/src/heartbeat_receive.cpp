/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-03 17:46:29
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-15 14:33:20
 */

#include "ros/ros.h"
#include "socket/heartbeat_receiver.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "heartbeat_receive_test");
    ros::NodeHandle nh, pnh("~");
    ros::Rate rate(50);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::State CURRENT_STATE;
    const std::string CURRENT_STATE_TOPIC = "mavros/state";

    ros::Subscriber current_state_sub = nh.subscribe<mavros_msgs::State>(
        CURRENT_STATE_TOPIC, 1,
        [&CURRENT_STATE](const mavros_msgs::State::ConstPtr &msg_ptr)
        {
            CURRENT_STATE = *msg_ptr;
            if (msg_ptr->mode != mavros_msgs::State::MODE_APM_COPTER_LAND &&
                msg_ptr->mode != mavros_msgs::State::MODE_APM_COPTER_BRAKE)
            {
            }
        });

    double land_timeout, hover_timeout;
    if (!pnh.getParam("land_timeout", land_timeout) ||
        !pnh.getParam("hover_timeout", hover_timeout))
    {
        ROS_ERROR("[heartbeat_receive] parameters not properly set");
        exit(1);
    }

    HeartbeatReceiver receiver(9999, land_timeout, hover_timeout);

    mavros_msgs::SetMode set_mode;

    while (ros::ok())
    {
        receiver.listenForHeartbeats();
        if (receiver.shouldLand())
        {
            std::cout << "[" << ros::Time::now() << "] "
                      << "Communication Lost, Landing..." << std::endl;
            if (CURRENT_STATE.mode != mavros_msgs::State::MODE_APM_COPTER_LAND)
            {
                set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_LAND;
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    ROS_INFO("Land enabled");
                }
            }

            if (!CURRENT_STATE.armed)
            {
                break;
            }
        }
        else if (receiver.shouldHover())
        {
            std::cout << "[" << ros::Time::now() << "] "
                      << "Communication Delay, Hovering..." << std::endl;
            if (CURRENT_STATE.mode != mavros_msgs::State::MODE_APM_COPTER_BRAKE)
            {
                set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_BRAKE;
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    ROS_INFO("Hover enabled");
                }
            }
        }
        else
        {
            std::cout << "[" << ros::Time::now() << "] " << "Communication Normal! Current mode is " 
                      << "[" << CURRENT_STATE.mode << "]" << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}