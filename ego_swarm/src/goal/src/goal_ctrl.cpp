#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <quadrotor_msgs/GoalSet.h>

#define IDLE        0
#define EXPLORE     1
#define RETURN      2
#define RESPAWN     3

ros::Publisher goal_pub, cur_tar_pub, mission_reset_pub, goal_state_pub;
ros::Subscriber odom_sub, rc_sub, replan_state_sub, occurred_error_sub, state_sub, waypoints_sub, ego_heartbeat_sub;
ros::ServiceClient set_mode_client;
ros::Time heartbeat_time, respawn_time;

geometry_msgs::Point current_odom_position, cur_tar_pt;
geometry_msgs::PoseArray waypointList;

quadrotor_msgs::GoalSet current_goal;
std_msgs::Int32 goal_state;

mavros_msgs::State current_state;

int explore_state = IDLE;
int loop_time = 0;

int last_confirm = 1900;
int threshold_confirm = 400;

int wp_num = 0;
int wp_index = 0;

bool first_flag = true;
bool trigger = false;
bool replan_state_flag = true;
bool error_flag = false;
bool sent_flag = false;
bool first_respawn_flag = true;

double hb_timeout =0.5;

void mavrosStateCallback(mavros_msgs::StateConstPtr msg)
{
    current_state = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odom_position = msg->pose.pose.position;
}

void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    if(msg->channels.size() < 16) return;
    int confirm = msg->channels[8];
    if(first_flag)
    {
        last_confirm = confirm;
        first_flag = false;
    }
    if( std::abs(confirm - last_confirm) < threshold_confirm) trigger = false;
    else
    {
        trigger = true;
        first_flag = true;
    }
}

void replanCallback(const std_msgs::Bool::ConstPtr& msg)
{
    replan_state_flag = msg->data;
}

void errorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    error_flag = msg->data;
}

void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    waypointList = *msg;
}

void heartbeatCallback(const std_msgs::Empty::ConstPtr& msg)
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

bool achieved(quadrotor_msgs::GoalSet goal)
{
    double delta_x = current_odom_position.x - goal.goal[0];
    double delta_y = current_odom_position.y - goal.goal[1];
    double delta_z = current_odom_position.z - goal.goal[2];
    double error = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
    if(error < 0.3) return true;
    else return false;
}

void SetLandMode()
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = mavros_msgs::State::MODE_APM_COPTER_LAND;
    if(current_state.mode != mavros_msgs::State::MODE_APM_COPTER_LAND && loop_time >= 20)
    {
        if( set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
        {
            ROS_INFO("LAND mode enabled");
        }
    }
}

void exploreFSM()
{
    if(loop_time < 20) loop_time ++;
    else loop_time = 0;

    switch(explore_state)
    {
        case IDLE:
        {
            if(loop_time >= 20)
            {
                std::cout << "State: IDLE. Waiting for ";
                if(current_state.mode != mavros_msgs::State::MODE_APM_COPTER_GUIDED) std::cout << "GUIDE mode, ";
                if(waypointList.poses.size() == 0) std::cout << "WaypointList, ";
                if(!trigger) std::cout << "trigger." << std::endl;
            }
            if(current_state.mode != mavros_msgs::State::MODE_APM_COPTER_GUIDED || waypointList.poses.size() == 0 || !trigger)
            {
                explore_state = IDLE;
            }
            else
            {
                explore_state = EXPLORE;
                replan_state_flag = true;
                error_flag = false;
                sent_flag = false;
                std::cout << "\033[32mIDLE -> EXPLORE\033[0m" << std::endl;
            }

            break;
        }

        case EXPLORE:
        {
            wp_num = waypointList.poses.size();
            current_goal.goal[0] = waypointList.poses[wp_index].position.x;
            current_goal.goal[1] = waypointList.poses[wp_index].position.y;
            current_goal.goal[2] = waypointList.poses[wp_index].position.z;

            if(loop_time >= 20)
            {
                std::cout << "State: EXPLORE. Current goal: [" 
                          << current_goal.goal[0] 
                          << ", " << current_goal.goal[1] 
                          << ", " << current_goal.goal[2] 
                          << "]" << std::endl;
            }
            
            if(!sent_flag)
            {
                goal_pub.publish(current_goal);
                sent_flag = true;
            }

            if(wp_index < wp_num && achieved(current_goal))
            {
                wp_index ++;
                sent_flag = false;
            }
            
            if(wp_index == wp_num || !replan_state_flag || error_flag)
            {
                explore_state = RETURN;
                replan_state_flag = true;
                error_flag = false;
                sent_flag = false;
                if(wp_index == wp_num) std::cout << "\033[32mFinal waypoint achieved. EXPLORE -> RETURN\033[0m" << std::endl;
                if(!replan_state_flag) std::cout << "\033[31mReplan Failed. EXPLORE -> RETURN\033[0m" << std::endl;
                if(error_flag) std::cout << "\033[31mError occurred in EGO Planner. EXPLORE -> RETURN\033[0m" << std::endl;
            }
            else {
                explore_state = EXPLORE;
            }

            if(!heartbeat())
            {
                explore_state = RESPAWN;
                replan_state_flag = true;
                error_flag = false;
                sent_flag = false;
                std::cout << "\033[31mEGO Planner Died. EXPLORE -> RESPAWN\033[0m" << std::endl;
            }

            break;
        }

        case RETURN:
        {
            if(wp_index == wp_num) wp_index --;
            if(wp_index > 0)
            {
                current_goal.goal[0] = waypointList.poses[wp_index - 1].position.x;
                current_goal.goal[1] = waypointList.poses[wp_index - 1].position.y;
                current_goal.goal[2] = waypointList.poses[wp_index - 1].position.z;
            }
            else if(wp_index == 0)
            {
                current_goal.goal[0] = 0.0;
                current_goal.goal[1] = 0.0;
                current_goal.goal[2] = 1.0;
            }

            if(loop_time >= 20)
            {
                std::cout << "State: RETURN. Current goal: [" 
                          << current_goal.goal[0] 
                          << ", " << current_goal.goal[1] 
                          << ", " << current_goal.goal[2] 
                          << "]" << std::endl;
            }

            if(!sent_flag)
            {
                goal_pub.publish(current_goal);
                sent_flag = true;
            }

            if(wp_index >= 0 && achieved(current_goal))
            {
                wp_index --;
                if(wp_index >= 0) sent_flag = false;
            }

            if(wp_index < 0)
            {
                SetLandMode();
            }

            if(current_state.mode == mavros_msgs::State::MODE_APM_COPTER_LAND)
            {
                explore_state = IDLE;
                waypointList.poses.clear();
                wp_index = 0;
                std::cout << "\033[32mReturned to launch. Rofly landing. RETURN -> IDLE\033[0m" << std::endl;
                
                std_msgs::Bool mission_reset_flag;
                mission_reset_flag.data = true;
                mission_reset_pub.publish(mission_reset_flag);
                std::cout << "\033[32mMISSION RESET!!!\033[0m" << std::endl;
            }
            else explore_state = RETURN;

            if(!heartbeat())
            {
                explore_state = RESPAWN;
                sent_flag = false;
                std::cout << "\033[31mEGO Planner Died. RETURN -> RESPAWN\033[0m" << std::endl;
            }

            break;
        }

        case RESPAWN:
        {         
            if(first_respawn_flag)
            {
                respawn_time = ros::Time::now();
                first_respawn_flag = false;
            }
            ros::Time current_time = ros::Time::now();
            if((current_time - respawn_time).toSec() > 5.0)
            {
                system_command_egorespawn();
                respawn_time = ros::Time::now();
            }
            if((current_time - respawn_time).toSec() > 3.0 && heartbeat())
            {
                explore_state = RETURN;
                first_respawn_flag = true;
                std::cout << "\033[32mEGO Planner respawn succeeded. RESPAWN -> RETURN\033[0m" << std::endl;
            }
            else explore_state = RESPAWN; 
            
            break;
        }
    }
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "goal_ctrl");
    ros::NodeHandle nh;

    goal_pub = nh.advertise<quadrotor_msgs::GoalSet>("/goal", 1);
    cur_tar_pub = nh.advertise<geometry_msgs::Point>("/cur_tar_pt", 1);
    mission_reset_pub = nh.advertise<std_msgs::Bool>("/mission_reset_signal", 1);
    goal_state_pub = nh.advertise<std_msgs::Int32>("/goal_state", 1);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odomCallback);
    rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1, rcInCallback);
    replan_state_sub = nh.subscribe<std_msgs::Bool>("/replan_state_flag", 1, replanCallback);
    occurred_error_sub = nh.subscribe<std_msgs::Bool>("/occurred_error_flag", 1, errorCallback);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, mavrosStateCallback);
    waypoints_sub = nh.subscribe<geometry_msgs::PoseArray>("/waypointList", 1, waypointsCallback);
    ego_heartbeat_sub = nh.subscribe<std_msgs::Empty>("/drone_0_ego_planner_node/planning/heartbeat", 1, heartbeatCallback);

    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        exploreFSM();

        goal_state.data = explore_state;
        goal_state_pub.publish(goal_state);

        cur_tar_pt.x = current_goal.goal[0];
        cur_tar_pt.y = current_goal.goal[1];
        cur_tar_pt.z = current_goal.goal[2];
        cur_tar_pub.publish(cur_tar_pt);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
