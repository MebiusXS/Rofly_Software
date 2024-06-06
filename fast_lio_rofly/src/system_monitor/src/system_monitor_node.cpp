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
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int imu_count = 0;
int odom_count = 0;
int rc_count = 0;
int egohb_count = 0;

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

double vel_lim = 0;
double acc_lim = 0;

string ToString(double val)
{
    stringstream ss;
    ss << setiosflags(ios::fixed) << setprecision(2) << val;
    string str = ss.str();
    return str;
}

void RCCallback(const mavros_msgs::RCIn::ConstPtr &msg)
{
    rc_count++;
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_count++;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_count++;
}

void egohbCallback(const std_msgs::Empty::ConstPtr &msg)
{
    egohb_count++;
}

void VACallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if(msg->data.size() < 2) return;
    vel_lim = msg->data[0];
    acc_lim = msg->data[1];
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "system_monitor_node");
    ros::NodeHandle nh;

    nh.param<int>("/system_monitor_node/text_width", text_width, 200);
    nh.param<int>("/system_monitor_node/text_height", text_height, 200);
    nh.param<int>("/system_monitor_node/text_left", text_left, 10);
    nh.param<int>("/system_monitor_node/text_top", text_top, 10);
    nh.param<int>("/system_monitor_node/text_size", text_size, 12);
    nh.param<int>("/system_monitor_node/line_width", text_line_width, 2);
    nh.param<string>("/system_monitor_node/text_font", text_font, "DejaVu Sans Mono");
    nh.param<double>("/system_monitor_node/fg_color_r", fg_color_r, 255.0);
    nh.param<double>("/system_monitor_node/fg_color_g", fg_color_g, 255.0);
    nh.param<double>("/system_monitor_node/fg_color_b", fg_color_b, 255.0);
    nh.param<double>("/system_monitor_node/fg_color_a", fg_color_a, 1.0);
    nh.param<double>("/system_monitor_node/bg_color_r", bg_color_r, 0.0);
    nh.param<double>("/system_monitor_node/bg_color_g", bg_color_g, 0.0);
    nh.param<double>("/system_monitor_node/bg_color_b", bg_color_b, 0.0);
    nh.param<double>("/system_monitor_node/bg_color_a", bg_color_a, 0.2);

    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 1000, RCCallback);
    ros::Subscriber cur_odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry_imu", 1000, OdomCallback);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 1000, imuCallback);
    ros::Subscriber egohb_sub = nh.subscribe<std_msgs::Empty>("/drone_0_ego_planner_node/planning/heartbeat", 1000, egohbCallback);
    ros::Subscriber vel_acc_sub = nh.subscribe<std_msgs::Float64MultiArray>("/drone_0_ego_planner_node/planning/vel_acc_vec", 1000, VACallback);

    ros::Publisher text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/system_state", 1000);


    ros::Rate looprate(1);

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
        string content = "IMU FPS: ";
        content.append(ToString(imu_count));
        content.append("\nOdom FPS: ");
        content.append(ToString(odom_count));
        content.append("\nRC FPS: ");
        content.append(ToString(rc_count));
        content.append("\nEGO HB: ");
        content.append(ToString(egohb_count));
        content.append("\nEGO maxVel: ");
        content.append(ToString(vel_lim));
        content.append("\nEGO maxAcc: ");
        content.append(ToString(acc_lim));

        text.text = content;

        text_pub.publish(text);

        imu_count = 0;
        odom_count = 0;
        rc_count = 0;
        egohb_count = 0;

        ros::spinOnce();
        looprate.sleep();
    }
}
