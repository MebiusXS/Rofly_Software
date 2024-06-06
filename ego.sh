#!/bin/bash

source ~/ego_swarm/devel/setup.bash
roslaunch ego_planner run_in_exp.launch & sleep 3;
roslaunch raycast raycast.launch & sleep 3;
# roslaunch goal goal_ctrl.launch & sleep 1;
roslaunch goal_rc goal_rc.launch & sleep 1;
wait;
