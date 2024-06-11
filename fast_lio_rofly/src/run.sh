#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash

roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5

source ~/fast_lio_rofly/devel/setup.bash
roslaunch fast_lio mapping_avia.launch & sleep 5
# roslaunch ekf nokov.launch

wait
exec /bin/bash
 
