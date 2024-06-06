#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/RCIn.h>  // mavros/rc/in话题对应的消息类型
#include <mavros_msgs/OverrideRCIn.h>  // mavros/rc/override话题对应的消息类型
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Float64MultiArray.h>

geometry_msgs::Point current_odom_position;
geometry_msgs::Quaternion current_orientation;
quadrotor_msgs::GoalSet goal;
std_msgs::Float64MultiArray vel_acc;

ros::Publisher pub;
ros::Publisher cur_tar_pub;
ros::Publisher vel_acc_pub;
bool first_flag = true;
mavros_msgs::State current_state;

ros::Time heartbeat_time;

double max_step = 2.0; // m/s
int deadband = 100;
double goal_duration = 1.0;
int rate = 10;
int last_chan_ego = 0;

double low_vel = 0.5;
double mid_vel = 1.5;
double high_vel = 3.0;
double low_acc = 1.0;
double mid_acc = 3.0;
double high_acc = 6.0;

double hb_timeout =0.5;

void mavrosStateCallback(mavros_msgs::StateConstPtr msg)
{
    current_state = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_position.x = msg->pose.pose.position.x;
    current_odom_position.y = msg->pose.pose.position.y;
    current_odom_position.z = msg->pose.pose.position.z;
    current_orientation = msg->pose.pose.orientation;
}

double getStep(int chan)
{
    double step = 0;
    if(chan >= 1495 - deadband/2 && chan <= 1495 + deadband/2) step = 0;
    else step = max_step / rate * (chan - 1495) / 450;
    return step;
}

void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    if(msg->channels.size() < 16) return;

    int chan_lr = msg->channels[0];
    int chan_fb = msg->channels[1];
    int chan_ud = msg->channels[2];
    int chan_md = msg->channels[4]; // mode.
    int chan_ego = msg->channels[5]; // ego vel&acc adjust

    if(chan_md > 1700)
    {
        if(first_flag){
            first_flag = false;
            goal.goal[0] = current_odom_position.x;
            goal.goal[1] = current_odom_position.y;
            goal.goal[2] = current_odom_position.z;
        }
            // tf2::Quaternion quat;
            // tf2::convert(current_orientation, quat);
            // tf2::Matrix3x3 m(quat);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);//提取欧拉角
            double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0;

            delta_x -= getStep(chan_fb);
            delta_y -= getStep(chan_lr);
            delta_z += getStep(chan_ud);

            // double global_x = delta_x * cos(yaw) - delta_y * sin(yaw);
            // double global_y = delta_x * sin(yaw) + delta_y * cos(yaw);
            // double global_z = delta_z;
            // goal.goal[0] += global_x;
            // goal.goal[1] += global_y;
            // goal.goal[2] += global_z;
            goal.goal[0] += delta_x;
            goal.goal[1] += delta_y;
            goal.goal[2] += delta_z;

    }
    else first_flag = true;
    // std::cout << "goal: " << goal.goal[0] << ", " << goal.goal[1] << ", " << goal.goal[2] << std::endl;
    vel_acc.data.resize(2);
    if(chan_ego < 1345){
        vel_acc.data[0] = low_vel;
        vel_acc.data[1] = low_acc;
    }else if(chan_ego > 1345 && chan_ego < 1645){
        vel_acc.data[0] = mid_vel;
        vel_acc.data[1] = mid_acc;
    }else if(chan_ego > 1645){
        vel_acc.data[0] = high_vel;
        vel_acc.data[1] = high_acc;
    }
    if(chan_ego != last_chan_ego){
        vel_acc_pub.publish(vel_acc);
        last_chan_ego = chan_ego;
    }
}

bool achieveJudge()
{
    double error_x = current_odom_position.x - goal.goal[0];
    double error_y = current_odom_position.y - goal.goal[1];
    double error_z = current_odom_position.z - goal.goal[2];
    double error = sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2));
    if(error < 0.3) return true;
    else return false;
}

void timerCallback(const ros::TimerEvent& e)
{
    goal.drone_id = 0;
    if(current_state.mode == "GUIDED" && !first_flag && !achieveJudge()) {
        pub.publish(goal);

        geometry_msgs::Point cur_tar_pt;
        cur_tar_pt.x = goal.goal[0];
        cur_tar_pt.y = goal.goal[1];
        cur_tar_pt.z = goal.goal[2];
        cur_tar_pub.publish(cur_tar_pt);
    }
}

void egohbCallback(const std_msgs::Empty::ConstPtr& msg)
{
    heartbeat_time = ros::Time::now();
}

bool heartbeat()
{
    ros::Time time_now = ros::Time::now();
    if((time_now - heartbeat_time).toSec() < hb_timeout) return true;
    else return false;
}

void system_command_egorespawn()
{
    int ret = system("bash /home/fast/ego_swarm/src/ego_respawn.sh");
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "goal_rc_node");
    ros::NodeHandle nh;

    nh.param("/goal_rc_node/max_step", max_step, 2.0);
    nh.param("/goal_rc_node/deadband", deadband, 100);
    nh.param("/goal_rc_node/goal_duration", goal_duration, 1.0);
    nh.param("/goal_rc_node/low_vel_limit", low_vel, 0.5);
    nh.param("/goal_rc_node/mid_vel_limit", mid_vel, 1.5);
    nh.param("/goal_rc_node/high_vel_limit", high_vel, 3.0);
    nh.param("/goal_rc_node/low_acc_limit", low_acc, 1.0);
    nh.param("/goal_rc_node/mid_acc_limit", mid_acc, 3.0);
    nh.param("/goal_rc_node/high_acc_limit", high_acc, 6.0);

    std::cout << "max_step = " << max_step << std::endl;
    std::cout << "deadband = " << deadband << std::endl;
    std::cout << "goal_duration = " << goal_duration << std::endl;

    pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal", 1);
    cur_tar_pub = nh.advertise<geometry_msgs::Point>("/cur_tar_pt", 1);
    vel_acc_pub = nh.advertise<std_msgs::Float64MultiArray>("/drone_0_ego_planner_node/planning/vel_acc_vec", 1, true);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("preview_goal_marker", 1);
    ros::Subscriber OdomSub = nh.subscribe("/Odometry_imu", 1, odomCallback);
    ros::Subscriber rcInSub = nh.subscribe("/mavros/rc/in", 1, rcInCallback);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, mavrosStateCallback);
    ros::Subscriber ego_hb_sub = nh.subscribe<std_msgs::Empty>("/drone_0_ego_planner_node/planning/heartbeat", 1, egohbCallback);

    ros::Timer goal_timer = nh.createTimer(ros::Duration(goal_duration), timerCallback);

    ros::Time last_respawn = ros::Time::now();
    bool first_respawn_flag = false;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "camera_init";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        marker.header.stamp = ros::Time::now();
        if (current_state.mode != "GUIDED") {
            marker.pose.position.x = current_odom_position.x;
            marker.pose.position.y = current_odom_position.y;
            marker.pose.position.z = current_odom_position.z;
        }
        else {
            marker.pose.position.x = goal.goal[0];
            marker.pose.position.y = goal.goal[1];
            marker.pose.position.z = goal.goal[2];
        }

        marker_pub.publish(marker);

        if(!heartbeat() && !first_respawn_flag){
            system_command_egorespawn();
            ROS_WARN("EGO Planner has died! Respawning...");
            last_respawn = ros::Time::now();
            first_respawn_flag = true;
        }else if(first_respawn_flag){
            if((ros::Time::now() - last_respawn).toSec() < 5.0) first_respawn_flag = true;
            else first_respawn_flag = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}