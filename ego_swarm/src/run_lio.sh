#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ego_swarm/devel/setup.bash

roslaunch px4ctrl run_ctrl.launch & sleep 1 # apmctrl package is also named "px4ctrl"!

# planner
# roslaunch ego_planner run_in_exp.launch & sleep 1

wait
exec /bin/bash
