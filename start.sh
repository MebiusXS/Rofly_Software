#!/bin/bash
expect_sh="/home/fast/expect.sh"
pcviz_sh="/home/fast/pc_viz.sh"
kcon_sh="/home/fast/kcon.sh"
ego_sh="/home/fast/ego.sh"
bag_sh="/home/fast/bag.sh"
posctrl_sh="/home/fast/posctrl.sh"
fastlio_sh="/home/fast/fastlio.sh"

# mavros
sleep 1
xdotool key Shift+Ctrl+O
sleep 1
xdotool key Alt+Up
sleep 1
xdotool key Shift+Ctrl+E
sleep 1
xdotool type $expect_sh
xdotool key Return
sleep 3

# pointcloud visualization
xdotool key Alt+Left
sleep 1
xdotool key Shift+Ctrl+O
# sleep 1
# xdotool type $pcviz_sh
# xdotool key Return
sleep 1

# keyboard
xdotool key Alt+Right
sleep 1
xdotool key Shift+Ctrl+O
# sleep 1
# xdotool type $kcon_sh
# xdotool key Return
sleep 1

# goal_ctrl ego
xdotool key Alt+Down
sleep 1
xdotool key Shift+Ctrl+E
sleep 1
xdotool key Alt+Left
sleep 1
xdotool type $ego_sh
xdotool key Return
sleep 1

# rosbag
xdotool key Shift+Ctrl+O
sleep 1
xdotool key Alt+Right
sleep 1
xdotool type $bag_sh
xdotool key Return
sleep 1

# livox fastlio usbcam rviz rviz_text rqt_reconfigure# posctrl
xdotool key Shift+Ctrl+O
sleep 1
xdotool key Alt+Left
sleep 1
xdotool type $posctrl_sh
xdotool key Return
sleep 1

# livox fastlio usbcam rviz rviz_text rqt_reconfigure
xdotool key Alt+Right
sleep 1
xdotool type $fastlio_sh
xdotool key Return

exit
