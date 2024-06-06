#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/ego_swarm/devel/setup.bash

# goal
roslaunch goal goal_listener.launch
roslaunch goal rc_ctrl.launch

wait
exec /bin/bash