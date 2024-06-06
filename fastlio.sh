#!/bin/bash

source ~/livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch & sleep 2;

source ~/cam_ws/devel/setup.bash
roslaunch usb_cam usb_cam.launch & sleep 1;

source ~/fast_lio_rofly/devel/setup.bash
roslaunch fast_lio mapping_mid360.launch & sleep 2;
roslaunch odom_frequency_conversion odom_frequency_conversion.launch & sleep 2;
roslaunch rviz_text rviz_text.launch & sleep 2;
roslaunch system_monitor system_monitor.launch & sleep 2;
roslaunch fast_lio rviz.launch & sleep 1;

# rosrun rqt_reconfigure rqt_reconfigure & sleep 1;

wait;
