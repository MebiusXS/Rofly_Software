# goal_listener
rviz发出的topic是/move_base_simple/goal，消息格式和ego-swarm用的不一样，所以这里监听目标点的话题，然后转了一下格式再发布出去
启动命令建议写在run_lio.sh里面
roslaunch goal goal_listener.launch;

# 键盘：keyboard_ctrl
roslaunch goal keyboard_ctrl.launch

方向定义：
        W/前           R/高
A/左    S/后    D/右    F/低

Enter回车（确定）

# 遥控：rc_ctrl.cpp
roslaunch goal rc_ctrl.launch
