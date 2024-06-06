#include "ros/ros.h"
#include "virtual_rc/keyboard_reader.h"
#include "virtual_rc/rc_control_publisher.h"

#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rc_test");
    ros::NodeHandle nh, pnh("~");

    KeyboardReader kbReader;
    RCControlPublisher rcPublisher(nh);

    ros::ServiceClient param_set_client = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");

    int roll, pitch, throttle, yaw;
    int rc_mid = 1495, rc_yaw_adjust = 50, rc_adjust = 100;

    throttle = 1045;
    roll = pitch = yaw = rc_mid;

    int sysid_mygcs = 255;

    RC_MODE mode = RC_MODE::AIRGROUND; // Loiter

    bool emergency = false;

    if (!pnh.getParam("rc_adjust", rc_adjust) ||
        !pnh.getParam("rc_yaw_adjust", rc_yaw_adjust))
    {
        ROS_ERROR("[rc_control] parameters not properly set");
        exit(1);
    }

    ros::Rate rate(10);

    while (ros::ok())
    {
        kbReader.update();

        if (kbReader.keyPressed('['))
        {
            mavros_msgs::ParamSet srv;
            srv.request.param_id = "SYSID_MYGCS";
            srv.request.value.integer = 1;
            srv.request.value.real = 0.0;

            if (param_set_client.call(srv))
            {
                if (srv.response.success)
                {
                    sysid_mygcs = 1;
                }
            }
        }

        if (kbReader.keyPressed(']'))
        {
            mavros_msgs::ParamSet srv;
            srv.request.param_id = "SYSID_MYGCS";
            srv.request.value.integer = 255;
            srv.request.value.real = 0.0;

            if (param_set_client.call(srv))
            {
                if (srv.response.success)
                {
                    sysid_mygcs = 255;
                }
            }
        }

        // 前进(i)、后退(k)、左(j)、右(l)
        pitch = kbReader.keyPressed('i') ? (rc_mid - rc_adjust) : (kbReader.keyPressed('k') ? (rc_mid + rc_adjust) : rc_mid);
        roll = kbReader.keyPressed('j') ? (rc_mid - rc_adjust) : (kbReader.keyPressed('l') ? (rc_mid + rc_adjust) : rc_mid);

        // 增加油门(w)、减少油门(s)、一键回中(g)
        if (kbReader.keyPressed('w'))
        {
            throttle = std::min(throttle + 50, 1945);
        }

        if (kbReader.keyPressed('s'))
        {
            throttle = std::max(throttle - 50, 1045);
        }

        if (kbReader.keyPressed('g'))
        {
            throttle = rc_mid;
        }

        // 左偏航(a)、右偏航(d)
        yaw = kbReader.keyPressed('a') ? (rc_mid - rc_yaw_adjust) : (kbReader.keyPressed('d') ? (rc_mid + rc_yaw_adjust) : rc_mid);

        // 切换模式：自稳模式(1)、空地模式(2)、自动模式(3)
        if (kbReader.keyPressed('1'))
            mode = RC_MODE::STABILIZE;

        if (kbReader.keyPressed('2'))
            mode = RC_MODE::AIRGROUND;

        if (kbReader.keyPressed('3'))
            mode = RC_MODE::AUTOMATION;

        // 急停(0)、取消急停(9)
        if (kbReader.keyPressed('0'))
            emergency = true;

        if (kbReader.keyPressed('9'))
            emergency = false;

        // 调节rc_adjust
        if (kbReader.keyPressed('='))
        {
            rc_adjust = std::min(rc_adjust + 10, 450);
            rc_yaw_adjust = std::min(rc_yaw_adjust + 10, 450);
        }

        if (kbReader.keyPressed('-'))
        {
            rc_adjust = std::max(rc_adjust - 10, 0);
            rc_yaw_adjust = std::max(rc_yaw_adjust - 10, 0);
        }

        rcPublisher.set_channels(roll, pitch, throttle, yaw, mode, emergency);
        rcPublisher.rc_publish();

        clear();
        printw("Roll: %i, Pitch: %i, Throttle: %i, Yaw: %i\n", roll, pitch, throttle, yaw);
        printw("Mode: %i, Emergency: %s\n", (int)mode, emergency ? "True" : "False");
        printw("RC_ADJUST: %i, RC_YAW_ADJUST: %i\n", rc_adjust, rc_yaw_adjust);
        printw("SYSID_MYGCS: %i\n", sysid_mygcs);
        refresh();

        kbReader.reset();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}