#!/bin/bash
source /opt/ros/noetic/setup.bash
sudo chmod 777 /dev/ttyTHS0
roslaunch mavros apm.launch & sleep 1;
# rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1
# rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1

# rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0 & sleep 1
# rosrun mavros mavcmd long 511 85 5000 0 0 0 0 0 & sleep 1
# rosrun mavros mavcmd long 511 84 5000 0 0 0 0 0 & sleep 1
# rosrun mavros mavcmd long 511 27 5000 0 0 0 0 0 & sleep 1
wait;
