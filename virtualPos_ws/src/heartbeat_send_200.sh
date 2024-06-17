#!/bin/bash
source /home/cheer/Documents/virtualPos_ws/devel/setup.bash;
roslaunch virtual_pos heartbeat_send_200.launch;
wait;