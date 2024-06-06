#include <jsk_rviz_plugins/OverlayText.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

using namespace std;

string flight_mode;
string arm_state;
string emergency_stop;
double voltage = 0;
double volt_percent = 0;
double tf_height = 0;
double tar_x = 0;
double tar_y = 0;
double tar_z = 0;

bool state_flag = false;
bool arm_flag = false;

int text_width = 0;
int text_height = 0;
int text_left = 0;
int text_top = 0;
int text_size = 0;
int text_line_width = 0;
string text_font;
double fg_color_r = 0;
double fg_color_g = 0;
double fg_color_b = 0;
double fg_color_a = 0;
double bg_color_r = 0;
double bg_color_g = 0;
double bg_color_b = 0;
double bg_color_a = 0;

geometry_msgs::Point cur_tar_pt;
nav_msgs::Odometry cur_odom;
visualization_msgs::Marker planned_path;

ros::Publisher planned_path_pub;

string ToString(double val)
{
    stringstream ss;
    ss << setiosflags(ios::fixed) << setprecision(2) << val;
    string str = ss.str();
    return str;
}

void system_command_shutdown()
{
    int ret = system("bash /home/fast/reboot.sh");
}

void StateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    if(msg->mode == "STABILIZE") flight_mode = "姿态模式";
    else if(msg->mode == "ACRO") flight_mode = "空地模式";
    else if(msg->mode == "LOITER") flight_mode = "位置模式";
    else if(msg->mode == "GUIDED_NOGPS" || msg->mode == "GUIDED") flight_mode = "自动模式";
    else flight_mode = msg->mode;

    if(msg->armed){
        arm_state = "飞行中";
        arm_flag = true;
    }else{
        arm_state = "已锁定";
        arm_flag = false;
    }

    state_flag = true;
}

void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
{
    voltage = msg->voltage;
    volt_percent = (voltage - 20.4) / 4.8 * 100;
}

void TF02Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    tf_height = msg->range;
}

void TargetCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    tar_x = msg->position.y / 100;
    tar_y = msg->position.x / 100;
    tar_z = -msg->position.z / 100;
}

void RCCallback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    if(msg->channels.size() < 16) return;
    if(msg->channels[15] == 1945 && !arm_flag) system_command_shutdown();
    if(msg->channels[6] == 1045) emergency_stop = "否";
    if(msg->channels[6] == 1945) emergency_stop = "是";
}

void cur_tarCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    cur_tar_pt = *msg;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    cur_odom = *msg;
}

void pathCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    planned_path = *msg;
    planned_path.header.frame_id = "camera_init";
    planned_path.header.stamp = ros::Time();
    planned_path_pub.publish(planned_path);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_text_node");
    ros::NodeHandle nh;

    nh.param<int>("/rviz_text_node/text_width", text_width, 200);
    nh.param<int>("/rviz_text_node/text_height", text_height, 200);
    nh.param<int>("/rviz_text_node/text_left", text_left, 10);
    nh.param<int>("/rviz_text_node/text_top", text_top, 10);
    nh.param<int>("/rviz_text_node/text_size", text_size, 12);
    nh.param<int>("/rviz_text_node/line_width", text_line_width, 2);
    nh.param<string>("/rviz_text_node/text_font", text_font, "DejaVu Sans Mono");
    nh.param<double>("/rviz_text_node/fg_color_r", fg_color_r, 255.0);
    nh.param<double>("/rviz_text_node/fg_color_g", fg_color_g, 255.0);
    nh.param<double>("/rviz_text_node/fg_color_b", fg_color_b, 255.0);
    nh.param<double>("/rviz_text_node/fg_color_a", fg_color_a, 1.0);
    nh.param<double>("/rviz_text_node/bg_color_r", bg_color_r, 0.0);
    nh.param<double>("/rviz_text_node/bg_color_g", bg_color_g, 0.0);
    nh.param<double>("/rviz_text_node/bg_color_b", bg_color_b, 0.0);
    nh.param<double>("/rviz_text_node/bg_color_a", bg_color_a, 0.2);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1000, StateCallback);
    ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 1000, BatteryCallback);
    ros::Subscriber tf_sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/rangefinder_pub", 1000, TF02Callback);
    ros::Subscriber target_sub = nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 1000, TargetCallback);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1000, RCCallback);
    ros::Subscriber cur_tar_pt_sub = nh.subscribe<geometry_msgs::Point>("/cur_tar_pt", 10, cur_tarCallback);
    ros::Subscriber cur_odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry_imu", 10, OdomCallback);
    ros::Subscriber planned_path_sub = nh.subscribe<visualization_msgs::Marker>("/drone_0_ego_planner_node/optimal_list", 1, pathCallback);

    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/drone_info", 1000);
    // ros::Publisher target_vis_pub = nh.advertise<visualization_msgs::Marker>("/acro_target_marker", 1000);

    planned_path_pub = nh.advertise<visualization_msgs::Marker>("/planned_path", 1);

    ros::Rate looprate(100);

    // visualization_msgs::Marker target_marker;

    jsk_rviz_plugins::OverlayText text;
    std_msgs::ColorRGBA fg_color;
    std_msgs::ColorRGBA bg_color;

    fg_color.r = fg_color_r / 255.0;
    fg_color.g = fg_color_g / 255.0;
    fg_color.b = fg_color_b / 255.0;
    fg_color.a = fg_color_a;

    bg_color.r = bg_color_r / 255.0;
    bg_color.g = bg_color_g / 255.0;
    bg_color.b = bg_color_b / 255.0;
    bg_color.a = bg_color_a;

    text.width = text_width;
    text.height = text_height;
    text.left = text_left;
    text.top = text_top;
    text.text_size = text_size;
    text.line_width = text_line_width;
    text.font = text_font;
    text.fg_color = fg_color;
    text.bg_color = bg_color;

    while(ros::ok())
    {
        if(!state_flag)
        {
            flight_mode = "接收中...";
            arm_state = "接收中...";
        }

        string content = "飞行模式: ";
        content.append(flight_mode.c_str());
        content.append("\n飞机状态: ");
        content.append(arm_state.c_str());
        content.append("\n急停状态: ");
        content.append(emergency_stop.c_str());
        content.append("\n<span style=\"color: red;\">电压: ");
        content.append(ToString(voltage));
        content.append("V</span>\n电量: ");
        content.append(ToString(volt_percent));
        content.append("%\n<span style=\"color: red;\">离地高度: ");
        content.append(ToString(tf_height));
        content.append("m</span>\n目标点位置: ");
        content.append("\ntar_x = ");
        content.append(ToString(cur_tar_pt.x));
        content.append("m\ntar_y = ");
        content.append(ToString(cur_tar_pt.y));
        content.append("m\ntar_z = ");
        content.append(ToString(cur_tar_pt.z));
        content.append("m\n无人机位置: ");
        content.append("\ncur_x = ");
        content.append(ToString(cur_odom.pose.pose.position.x));
        content.append("m\ncur_y = ");
        content.append(ToString(cur_odom.pose.pose.position.y));
        content.append("m\ncur_z = ");
        content.append(ToString(cur_odom.pose.pose.position.z));
        content.append("m\n无人机速度: ");
        content.append("\nvel_x = ");
        content.append(ToString(cur_odom.twist.twist.linear.x));
        content.append("m/s\nvel_y = ");
        content.append(ToString(cur_odom.twist.twist.linear.y));
        content.append("m/s\nvel_z = ");
        content.append(ToString(cur_odom.twist.twist.linear.z));
        content.append("m/s");

        text.text = content;

        text_pub.publish(text);

        // target_marker.header.frame_id = "camera_init";
        // target_marker.header.stamp = ros::Time();
        // target_marker.ns = "my_namespace";
        // target_marker.id = 0;
        // target_marker.type = visualization_msgs::Marker::SPHERE;
        // target_marker.action = visualization_msgs::Marker::ADD;
        // target_marker.pose.position.x = tar_x;
        // target_marker.pose.position.y = tar_y;
        // target_marker.pose.position.z = tar_z;
        // target_marker.pose.orientation.w = 1;
        // target_marker.pose.orientation.x = 0;
        // target_marker.pose.orientation.y = 0;
        // target_marker.pose.orientation.z = 0;
        // target_marker.scale.x = 0.1;
        // target_marker.scale.y = 0.1;
        // target_marker.scale.z = 0.1;
        // target_marker.color.a = 1.0; // Don't forget to set the alpha!
        // target_marker.color.r = 0.0;
        // target_marker.color.g = 255.0;
        // target_marker.color.b = 0.0;

        // target_vis_pub.publish(target_marker);

        ros::spinOnce();
        looprate.sleep();

    }

}