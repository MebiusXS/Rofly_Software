//
// Created by pf on 2024/4/7.
//
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <ros/transport_hints.h>
#include <thread>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// int pubFrameRate = 200;
int odom_freq = 200;
ros::Publisher interpolated_pub,sleep_pub1,sleep_pub2;
// ros::Publisher  visionpose_pub, visionspeed_pub;
ros::Publisher  visionpose_pub;
bool isInitialized = false;
bool odom_tag = false;
std::vector<Eigen::Quaterniond> quaternionPoints;

int windowSize = 6;
int i = 0;
Eigen::Vector3d linear_velocity;
Eigen::Vector3d angular_velocity;
std::queue<Eigen::Vector3d> posqueue;
std::queue<Eigen::Quaterniond> oriqueue;
double odom_timestamp;
nav_msgs::Odometry odom_prediction;
geometry_msgs::PoseStamped vision_pose;
// geometry_msgs::TwistStamped vision_speed;
std_msgs::Float64 sleep_msg1,sleep_msg2;
Eigen::Vector3d previous_position;
double previous_time;

double time_pub = 0.0;
double time_1 = 0.0;
double time_2 = 0.0;
double time_3 = 0.0;

namespace Eigen {
    template<typename T>

    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    Eigen::Quaterniond q_slerp_next(const double &t, const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2) {
        EIGEN_USING_STD_MATH(acos)
        EIGEN_USING_STD_MATH(sin)
        EIGEN_USING_STD_MATH(cos)
        const double one = double(1) - Eigen::NumTraits<double>::epsilon();
        double d = q1.dot(q2);
        double absD = Eigen::numext::abs(d);

        double scale0;
        double scale1;

        if (absD >= one) {
            scale0 = -t;
            scale1 = double(1) + t;
//            ROS_INFO_STREAM("theta" << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        } else {
            // theta is the angle between the 2 quaternions

            double theta = acos(absD);
//            ROS_INFO_STREAM("theta" << theta << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
            double sinTheta = sin(theta);

            scale0 = -sin((t * theta)) / sinTheta;
            scale1 = cos((t * theta)) + absD * sin((t * theta)) / sinTheta;
        }
        if (d < double(0)) scale1 = -scale1;

        Eigen::Quaterniond q_inter;

        q_inter.x() = scale0 * q1.x() + scale1 * q2.x();
        q_inter.y() = scale0 * q1.y() + scale1 * q2.y();
        q_inter.z() = scale0 * q1.z() + scale1 * q2.z();
        q_inter.w() = scale0 * q1.w() + scale1 * q2.w();
        q_inter.normalize();
//        ROS_INFO_STREAM("q_inter:" << q_inter.coeffs().transpose() << "\n");

        return q_inter;
    }
}
void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (posqueue.size() < windowSize || posqueue.empty()) {
        posqueue.push(Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        if(posqueue.size() == windowSize){
            odom_tag = true;
            odom_timestamp = ros::Time::now().toSec();
        }
    } else {
        odom_tag = true;
        odom_timestamp = ros::Time::now().toSec();
        posqueue.pop();
        posqueue.push(Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    }
    if (oriqueue.size() < windowSize || oriqueue.empty()) {
        Eigen::Quaterniond q_t(double(msg->pose.pose.orientation.w),double(msg->pose.pose.orientation.x),double(msg->pose.pose.orientation.y),double(msg->pose.pose.orientation.z));
        oriqueue.push(q_t);
    } else {
        oriqueue.pop();
        Eigen::Quaterniond q_t(double(msg->pose.pose.orientation.w),double(msg->pose.pose.orientation.x),double(msg->pose.pose.orientation.y),double(msg->pose.pose.orientation.z));
        oriqueue.push(q_t);
    }
}


void timer_cbk(const ros::TimerEvent &)
{
    if (posqueue.size()==windowSize) {
        time_1 = ros::Time::now().toSec();
        // int numInterpolatedPoints = 200;
        
        //double previous_time;
        auto dt = ros::Time::now().toSec() - odom_timestamp;

        // if(dt>=1.0){
        //     ROS_WARN("NO Odom");
        //     break;
        // }

        //std::cout << "dt: " << dt << std::endl;
        //Eigen::Vector3d previous_position;
        // ROS_INFO_STREAM("****************************************");
        double t = 1.0 + dt / (windowSize * 0.1);
        
        double t_ori = t-1;
        Eigen::Vector3d result(0.0, 0.0, 0.0);
        Eigen::Quaterniond result_q;
        std::queue<Eigen::Vector3d> temp_positionQueue = posqueue;
        std::queue<Eigen::Quaterniond> temp_oriQueue = oriqueue;
        while (temp_positionQueue.size() > 1) {
            std::queue<Eigen::Vector3d> new_positionQueue;
            std::queue<Eigen::Quaterniond> new_oriQueue;
            while (temp_positionQueue.size() > 1) {
                Eigen::Vector3d p0 = temp_positionQueue.front();
                Eigen::Quaterniond q0 = temp_oriQueue.front();
                temp_positionQueue.pop();
                temp_oriQueue.pop();
                Eigen::Vector3d p1 = temp_positionQueue.front();
                Eigen::Quaterniond q1 = temp_oriQueue.front();  
                Eigen::Vector3d interpolated_position = (1 - t) * p0 + t * p1;
                Eigen::Quaterniond interpolated_ori = Eigen::q_slerp_next(t_ori, q0, q1);
                new_positionQueue.push(interpolated_position);
                new_oriQueue.push(interpolated_ori);
            }
            temp_positionQueue = new_positionQueue;
            temp_oriQueue = new_oriQueue;
        }
        result = temp_positionQueue.front();
        result_q = temp_oriQueue.back();
        auto result_time = ros::Time::now().toSec();
        
        if ( !odom_tag ) {
            linear_velocity = (result - previous_position) / (result_time - previous_time);
            // auto x  = result - previous_position; 
            // auto y = result_time - previous_time;
            //std::cout << linear_velocity.x() << " = (" << result.x() << " - " << previous_position.x() << ") / " << y << std::endl;
            Eigen::Quaterniond q_dot = Eigen::q_slerp_next(t_ori + (result_time - previous_time), result_q, temp_oriQueue.back());
            Eigen::AngleAxisd delta_rotation(result_q.inverse() * q_dot);
            angular_velocity = delta_rotation.axis() * delta_rotation.angle() / (result_time - previous_time);
        }
        else {
            odom_tag = false;
        }

        previous_position = result;
        //std::cout << "previous_position: " << previous_position << " = " << "result: " << result << std::endl;
        previous_time = result_time;


        odom_prediction.header.stamp = ros::Time::now();
        odom_prediction.header.frame_id = "world";
        odom_prediction.child_frame_id = "body";
        odom_prediction.pose.pose.position.x = result(0);
        odom_prediction.pose.pose.position.y = result(1);
        odom_prediction.pose.pose.position.z = result(2);
        odom_prediction.pose.pose.orientation.x = result_q.x();
        odom_prediction.pose.pose.orientation.y = result_q.y();
        odom_prediction.pose.pose.orientation.z = result_q.z();
        odom_prediction.pose.pose.orientation.w = result_q.w();

        odom_prediction.twist.twist.linear.x = linear_velocity.x();
        odom_prediction.twist.twist.linear.y = linear_velocity.y();
        odom_prediction.twist.twist.linear.z = linear_velocity.z();
        odom_prediction.twist.twist.angular.x = angular_velocity.x();
        odom_prediction.twist.twist.angular.y = angular_velocity.y();
        odom_prediction.twist.twist.angular.z = angular_velocity.z();
        time_2 = ros::Time::now().toSec();
        time_pub = time_2 - time_1;
        sleep_msg1.data = time_pub;
        sleep_pub1.publish(sleep_msg1);

        // rate.sleep();

        time_3 = ros::Time::now().toSec();
        sleep_msg2.data = time_3 - time_2;
        sleep_pub2.publish(sleep_msg2);
        vision_pose.pose = odom_prediction.pose.pose;
        vision_pose.header = odom_prediction.header;
        // vision_speed.header = odom_prediction.header;
        // vision_speed.twist = odom_prediction.twist.twist;
        interpolated_pub.publish(odom_prediction);
        visionpose_pub.publish(vision_pose);
        // visionspeed_pub.publish(vision_speed);
        
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    nh.param("/odom_frequency_conversion/frequency", odom_freq, 200);

    bool imu = false;
    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>
            ("/Odometry", 10000, odometryCallback, ros::TransportHints().tcpNoDelay());
    visionpose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10000);
    // visionspeed_pub = nh.advertise<geometry_msgs::TwistStamped>
    //         ("/mavros/vision_speed/speed_twist", 10000);
    interpolated_pub = nh.advertise<nav_msgs::Odometry>
            ("/Odometry_imu", 10000);
    sleep_pub1 = nh.advertise<std_msgs::Float64>("/sleep_time1", 10000);
    sleep_pub2 = nh.advertise<std_msgs::Float64>("/sleep_time2", 10000);

    // if(imu){
    //     ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
    //         ("/mavros/imu/data", 10000, imuCallback, ros::TransportHints().tcpNoDelay());
    // }
    
    //std::thread publishThread(publishInterpolatedOdometry);

    ros::Timer timer = nh.createTimer(ros::Duration(1.0/odom_freq), timer_cbk);

    // ros::Rate rate(pubFrameRate);
    
    // while (ros::ok()) {
        
    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;
}
