/*
 * @Author: Xinwei Chen (xinweichen@zju.edu.cn)
 * @Date: 2024-04-11 14:54:37
 * @Last Modified by: Xinwei Chen
 * @Last Modified time: 2024-04-16 21:12:00
 */

#include "ros/ros.h"
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

#include "virtual_rc/keyboard_reader.h"
#include "virtual_rc/rc_control_publisher.h"
#include "virtual_pos/virtual_pos_publisher.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/CommandBool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mix_control");
    ros::NodeHandle nh, pnh("~");

    KeyboardReader kbReader;
    RCControlPublisher rcPublisher(nh);
    VirtualPosPublisher vtPosPublisher(nh);

    // Get parameters
    double request_duration;
    double move_x, move_y, move_z, move_yaw;

    if (!pnh.getParam("move_x", move_x) ||
        !pnh.getParam("move_y", move_y) ||
        !pnh.getParam("move_z", move_z) ||
        !pnh.getParam("move_yaw", move_yaw) ||
        !pnh.getParam("request_duration", request_duration))
    {
        ROS_ERROR("[mix_control] parameters not properly set");
        exit(1);
    }

    // Subscribe to the current state
    mavros_msgs::State CURRENT_STATE;
    const std::string CURRENT_STATE_TOPIC = "mavros/state";

    ros::Subscriber current_state_sub = nh.subscribe<mavros_msgs::State>(
        CURRENT_STATE_TOPIC, 1,
        [&CURRENT_STATE](const mavros_msgs::State::ConstPtr &msg_ptr)
        {
            CURRENT_STATE = *msg_ptr;
        });

    // Subscribe to the current pose
    bool GUIDED_FLAG = false;
    double CURRENT_YAW = 0.0, CURRENT_THETA = 0.0;

    geometry_msgs::PoseStamped CURRENT_POSE;
    const std::string CURRENT_POSE_TOPIC = "mavros/local_position/pose";

    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        CURRENT_POSE_TOPIC, 1,
        [&CURRENT_POSE, &GUIDED_FLAG, &CURRENT_YAW, &CURRENT_THETA](const geometry_msgs::PoseStamped::ConstPtr &msg_ptr)
        {
            if (!GUIDED_FLAG)
            {
                CURRENT_POSE = *msg_ptr;
                CURRENT_YAW = tf2::getYaw(CURRENT_POSE.pose.orientation);
            }
            CURRENT_THETA = tf2::getYaw(msg_ptr->pose.orientation);
        });

    // Mavros service clients
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

    // Mavros messages
    mavros_msgs::SetMode set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Mode flags
    bool ARM_NOW = false, LAND_NOW = false, GUIDED_NOW = false, AIRGROUND_NOW = false;

    /*============RC channels============*/
    bool EMERGENCY_NOW = false, TAKEOFF_NOW = false, RC_NOW = true;
    int roll = 1495, pitch = 1495, yaw = 1495, throttle = 1045;
    RC_MODE mode = RC_MODE::AIRGROUND;

    mavros_msgs::ParamSet srv;
    srv.request.param_id = "SYSID_MYGCS";
    srv.request.value.real = 0.0;
    /*===================================*/

    ros::Rate rate(50);
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        kbReader.update();

        if (CURRENT_STATE.mode == mavros_msgs::State::MODE_APM_COPTER_BRAKE)
        {
            ARM_NOW = false;
            LAND_NOW = false;
            GUIDED_NOW = false;
            GUIDED_FLAG = false;
            TAKEOFF_NOW = false;
            AIRGROUND_NOW = false;
        }

        if (CURRENT_STATE.mode == mavros_msgs::State::MODE_APM_COPTER_LAND)
        {
            ARM_NOW = false;
            LAND_NOW = true;
            GUIDED_NOW = false;
            GUIDED_FLAG = false;
            TAKEOFF_NOW = false;
            AIRGROUND_NOW = false;
        }

        if (CURRENT_STATE.mode == mavros_msgs::State::MODE_APM_COPTER_GUIDED)
        {
            ARM_NOW = false;
            LAND_NOW = false;
            GUIDED_NOW = true;
            GUIDED_FLAG = true;
            TAKEOFF_NOW = false;
            AIRGROUND_NOW = false;
        }

        if (CURRENT_STATE.mode == mavros_msgs::State::MODE_APM_COPTER_ACRO)
        {
            LAND_NOW = false;
            GUIDED_NOW = false;
            GUIDED_FLAG = false;
            AIRGROUND_NOW = true;
        }

        // GUIDED
        if (kbReader.keyPressed('1'))
        {
            if (CURRENT_STATE.mode != mavros_msgs::State::MODE_APM_COPTER_GUIDED &&
                (ros::Time::now() - last_request > ros::Duration(request_duration)))
            {
                set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_GUIDED;
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    ARM_NOW = false;
                    LAND_NOW = false;
                    GUIDED_NOW = true;
                    GUIDED_FLAG = true;
                    TAKEOFF_NOW = false;
                    AIRGROUND_NOW = false;
                    mode = RC_MODE::AUTOMATION;
                }
                last_request = ros::Time::now();
            }
        }

        // NO GUIDED (AIRGROUND)
        if (kbReader.keyPressed('2'))
        {
            if (CURRENT_STATE.mode != mavros_msgs::State::MODE_APM_COPTER_ACRO &&
                (ros::Time::now() - last_request > ros::Duration(request_duration)))
            {
                set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_ACRO;
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    LAND_NOW = false;
                    GUIDED_NOW = false;
                    GUIDED_FLAG = false;
                    AIRGROUND_NOW = true;
                    mode = RC_MODE::AIRGROUND;
                }
                last_request = ros::Time::now();
            }
        }

        // LAND
        if (kbReader.keyPressed('3'))
        {
            if (CURRENT_STATE.mode != mavros_msgs::State::MODE_APM_COPTER_LAND &&
                (ros::Time::now() - last_request > ros::Duration(request_duration)))
            {
                set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_LAND;
                if (set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    ARM_NOW = false;
                    LAND_NOW = true;
                    GUIDED_NOW = false;
                    GUIDED_FLAG = false;
                    TAKEOFF_NOW = false;
                    AIRGROUND_NOW = false;
                    throttle = 1045;
                }
                last_request = ros::Time::now();
            }
        }

        // ARM
        if (kbReader.keyPressed('6'))
        {
            if (!CURRENT_STATE.armed &&
                (ros::Time::now() - last_request > ros::Duration(request_duration)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ARM_NOW = true;
                }
                last_request = ros::Time::now();
            }
        }

        // TAKEOFF
        if (kbReader.keyPressed('t'))
        {
            if (RC_NOW)
            {
                srv.request.value.integer = 1;
                if (param_set_client.call(srv) && srv.response.success &&
                    AIRGROUND_NOW && throttle == 1045)
                {
                    RC_NOW = false;
                    TAKEOFF_NOW = true;
                    throttle = 1495;
                }
            }
            else
            {
                if (AIRGROUND_NOW && throttle == 1045)
                {
                    TAKEOFF_NOW = true;
                    throttle = 1495;
                }
            }
        }

        // EMERGENCY STOP
        if (kbReader.keyPressed('0'))
        {
            if (RC_NOW)
            {
                srv.request.value.integer = 1;
                if (param_set_client.call(srv) && srv.response.success)
                {
                    RC_NOW = false;
                    ARM_NOW = false;
                    GUIDED_FLAG = false;
                    TAKEOFF_NOW = false;
                    EMERGENCY_NOW = true;
                    throttle = 1045;
                }
            }
            else
            {
                ARM_NOW = false;
                TAKEOFF_NOW = false;
                EMERGENCY_NOW = true;
                throttle = 1045;
            }
        }

        // CANCEL EMERGENCY STOP
        if (kbReader.keyPressed('9'))
        {
            if (RC_NOW)
            {
                srv.request.value.integer = 1;
                if (param_set_client.call(srv) && srv.response.success)
                {
                    RC_NOW = false;
                    EMERGENCY_NOW = false;
                }
            }
            else
            {
                EMERGENCY_NOW = false;
            }
        }

        // Switch back to keyboard
        if (kbReader.keyPressed('['))
        {
            srv.request.value.integer = 1;
            if (param_set_client.call(srv) && srv.response.success)
            {
                RC_NOW = false;
            }
        }

        // Switch back to RC
        if (kbReader.keyPressed(']'))
        {
            srv.request.value.integer = 255;
            if (param_set_client.call(srv) && srv.response.success)
            {
                RC_NOW = true;
            }
        }

        // Forward(w), Backward(s), Turn Left(a), Turn Right(d)
        if (kbReader.keyPressed('w'))
        {
            CURRENT_POSE.pose.position.z += move_z;
        }

        if (kbReader.keyPressed('s'))
        {
            CURRENT_POSE.pose.position.z -= move_z;
        }

        if (kbReader.keyPressed('a'))
        {
            CURRENT_YAW += move_yaw / 180 * M_PI;
            CURRENT_YAW = vtPosPublisher.normalize_yaw(CURRENT_YAW);
        }

        if (kbReader.keyPressed('d'))
        {
            CURRENT_YAW -= move_yaw / 180 * M_PI;
            CURRENT_YAW = vtPosPublisher.normalize_yaw(CURRENT_YAW);
        }

        // Up(i), Down(k), Left(j), Right(l)
        if (kbReader.keyPressed('i'))
        {
            CURRENT_POSE.pose.position.x += move_x * cos(CURRENT_THETA);
            CURRENT_POSE.pose.position.y += move_y * sin(CURRENT_THETA);
        }

        if (kbReader.keyPressed('k'))
        {
            CURRENT_POSE.pose.position.x -= move_x * cos(CURRENT_THETA);
            CURRENT_POSE.pose.position.y -= move_y * sin(CURRENT_THETA);
        }

        if (kbReader.keyPressed('j'))
        {
            CURRENT_POSE.pose.position.y += move_y * cos(CURRENT_THETA);
            CURRENT_POSE.pose.position.x -= move_x * sin(CURRENT_THETA);
        }

        if (kbReader.keyPressed('l'))
        {
            CURRENT_POSE.pose.position.y -= move_y * cos(CURRENT_THETA);
            CURRENT_POSE.pose.position.x += move_x * sin(CURRENT_THETA);
        }

        // Ajdust move_x, move_y, move_z, move_yaw
        if (kbReader.keyPressed('+'))
        {
            move_x += 0.1;
            move_y += 0.1;
            move_z += 0.1;
            move_yaw += 10.0;
        }

        if (kbReader.keyPressed('-'))
        {
            move_x -= 0.1;
            move_y -= 0.1;
            move_z -= 0.1;
            move_yaw -= 10.0;
        }

        mavros_msgs::PositionTarget pose;
        pose.header.frame_id = "camera_init";
        pose.header.stamp = ros::Time::now();

        pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_VX |
                         mavros_msgs::PositionTarget::IGNORE_VY |
                         mavros_msgs::PositionTarget::IGNORE_VZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        pose.position.x = CURRENT_POSE.pose.position.x;
        pose.position.y = CURRENT_POSE.pose.position.y;
        pose.position.z = CURRENT_POSE.pose.position.z;
        pose.yaw = CURRENT_YAW;

        vtPosPublisher.set_pos(pose);
        vtPosPublisher.pos_publish();

        rcPublisher.set_channels(roll, pitch, throttle, yaw, mode, EMERGENCY_NOW);
        rcPublisher.rc_publish();

        clear();

        printw("[Pos]  X: %6.2f, Y: %6.2f, Z: %6.2f, Yaw: %6.2f\n",
               CURRENT_POSE.pose.position.x,
               CURRENT_POSE.pose.position.y,
               CURRENT_POSE.pose.position.z,
               CURRENT_YAW / M_PI * 180);

        printw("---------------------------------------------------\n");
        printw("[Move] X: %6.2f, Y: %6.2f, Z: %6.2f, Yaw: %6.2f\n",
               move_x, move_y, move_z, move_yaw);

        printw("---------------------------------------------------\n");
        printw("[Status] Arm: %s, Guided: %s, Airground: %s, Land: %s\n",
               ARM_NOW ? "Yes" : "No",
               GUIDED_NOW ? "Yes" : "No",
               AIRGROUND_NOW ? "Yes" : "No",
               LAND_NOW ? "Yes" : "No");

        printw("[Status] Emergency: %s, Takeoff: %s, RC: %s\n",
               EMERGENCY_NOW ? "Yes" : "No",
               TAKEOFF_NOW ? "Yes" : "No",
               RC_NOW ? "Yes" : "No");

        printw("[Status] %s\n", CURRENT_STATE.mode.c_str());

        refresh();

        kbReader.reset();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}