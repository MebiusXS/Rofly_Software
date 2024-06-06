#/bin/bash
source /opt/ros/noetic/setup.bash
source ~/ego_swarm/devel/setup.bash
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
exec /bin/bash
