#!/bin/bash
source /home/cheer/Documents/virtualPos_ws/devel/setup.bash;
roslaunch virtual_pos heartbeat_send_202.launch;
wait;