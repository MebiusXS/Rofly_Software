#!/bin/bash
source /home/fast/virtualPos_ws/devel/setup.bash;
roslaunch virtual_pos heartbeat_receive.launch;
wait;