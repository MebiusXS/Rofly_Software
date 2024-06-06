#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#define MANUAL_CTRL 0
#define AUTO_HOVER  1
#define CMD_CTRL    2

ros::Subscriber state_sub_;
ros::Subscriber odom_sub_;
ros::Subscriber setpoint_sub_;
ros::Publisher setpoint_pub_;
ros::Publisher ctrl_state_pub_;

mavros_msgs::State state;
mavros_msgs::PositionTarget pos_cmd;
nav_msgs::Odometry localOdom;
quadrotor_msgs::PositionCommand setpoint_cmd;
std_msgs::Int32 ctrl_state;

ros::Time last_rec;
double cmd_time_out = 1.0;

bool pose_sub_flag = true;
bool auto_mode = false;
bool first_flag = true;
bool spcmd_flag = false;

int exec_state = MANUAL_CTRL;

void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
{
    state = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    if(pose_sub_flag){
        localOdom = *msg;
    }
}

void setpointCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) 
{
        setpoint_cmd = *msg;
        spcmd_flag = true;
        last_rec = ros::Time::now();
}

bool catchupJudge()
{
    double error_x = localOdom.pose.pose.position.x - setpoint_cmd.position.x;
    double error_y = localOdom.pose.pose.position.y - setpoint_cmd.position.y;
    double error_z = localOdom.pose.pose.position.z - setpoint_cmd.position.z;
    double error = sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2));
    if(error < 0.3) return true;
    else return false;
}

void execFSM()
{
    switch(exec_state)
    {
        case MANUAL_CTRL:
        {
            if(!auto_mode) {
                exec_state = MANUAL_CTRL;
                first_flag = true;
                pose_sub_flag = true;
            }
            else {
                exec_state = AUTO_HOVER;
                ROS_INFO("\033[32mMANUAL_CTRL -> AUTO_HOVER\033[0m");
            }
            pos_cmd.header.frame_id = "world";
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
                                mavros_msgs::PositionTarget::IGNORE_YAW;
            pos_cmd.position = localOdom.pose.pose.position;
            setpoint_pub_.publish(pos_cmd);

            break;
        }

        case AUTO_HOVER:
        {
            if(first_flag){
                pose_sub_flag = false;
                first_flag = false;
            
                double roll, pitch, yaw;
                tf2::Quaternion quat;
                tf2::convert(localOdom.pose.pose.orientation, quat);
                tf2::Matrix3x3 m(quat);
                m.getRPY(roll, pitch, yaw);

                pos_cmd.header.frame_id = "world";
                pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                pos_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                pos_cmd.position = localOdom.pose.pose.position;
                // if(yaw >= -PI/2) pos_cmd.yaw = yaw + PI/2;
                // else pos_cmd.yaw = yaw + 5*PI/2;
                pos_cmd.yaw = yaw;
            }
            pos_cmd.header.stamp = ros::Time::now();
            setpoint_pub_.publish(pos_cmd);

            if(spcmd_flag && auto_mode && catchupJudge()){
                exec_state = CMD_CTRL;
                ROS_INFO("\033[32mAUTO_HOVER -> CMD_CTRL\033[0m");
            }
            else if(auto_mode){
                exec_state = AUTO_HOVER;
            }
            else if(!auto_mode){
                exec_state = MANUAL_CTRL;
                ROS_INFO("\033[32mAUTO_HOVER -> MANUAL_CTRL\033[0m");
            }

            break;
        }

        case CMD_CTRL:
        {
            pose_sub_flag = true;
            first_flag = true;

            pos_cmd.header.frame_id = "world";
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            pos_cmd.type_mask = 0;
            pos_cmd.position = setpoint_cmd.position;
            pos_cmd.velocity = setpoint_cmd.velocity;
            pos_cmd.acceleration_or_force = setpoint_cmd.acceleration;
            pos_cmd.yaw = setpoint_cmd.yaw;
            pos_cmd.yaw_rate = setpoint_cmd.yaw_dot;
            // if(setpoint_cmd.yaw >= -PI/2) pos_cmd.yaw = setpoint_cmd.yaw + PI/2;
            // else pos_cmd.yaw = setpoint_cmd.yaw + 5*PI;
            setpoint_pub_.publish(pos_cmd);

            if(spcmd_flag && auto_mode){
                exec_state = CMD_CTRL;
            }
            else if(!spcmd_flag && auto_mode){
                exec_state = AUTO_HOVER;
                ROS_INFO("\033[32mCMD_CTRL -> AUTO_HOVER\033[0m");
            }
            else if(!auto_mode){
                exec_state = MANUAL_CTRL;
                ROS_INFO("\033[32mCMD_CTRL -> MANUAL_CTRL\033[0m");
            }

            break;
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "posctrl_node");

    ros::NodeHandle nh_;

    state_sub_ = nh_.subscribe("/mavros/state", 10, stateCallback);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, odomCallback);
    setpoint_sub_ = nh_.subscribe("/setpoints_cmd", 10, setpointCallback);
    // setpoint_sub_ = nh_.subscribe("/planning/pos_cmd", 10, setpointCallback);
    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ctrl_state_pub_ = nh_.advertise<std_msgs::Int32>("/ctrl_state", 10);

    last_rec = ros::Time::now();

    ros::Rate rate(100);

    while (ros::ok()) {
        auto_mode = state.mode == mavros_msgs::State::MODE_APM_COPTER_GUIDED;
        // auto_mode = state.mode == mavros_msgs::State::MODE_PX4_OFFBOARD;

        execFSM();
        ctrl_state.data = exec_state;
        ctrl_state_pub_.publish(ctrl_state);

        if((ros::Time::now() - last_rec).toSec() > cmd_time_out){
            spcmd_flag = false;
            last_rec = ros::Time::now();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
