///**
// * the code implemented in C language style
// * Created by pf
// * Date：4/6/2024
// */
#include <iostream>
#include <Eigen/Core>
#include <deque>
#include <stack>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include "fast_lio/debug_paraConfig.h"

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

bool mission_reset = false;
bool cutting_method = true;// 默认true的时候是上下
geometry_msgs::PoseArray savedPoses;

bool bPoint_use_laser = true;
geometry_msgs::Pose current_pose;
//ros::Publisher markerPub;
ros::Publisher makerArrayPub;
ros::Publisher maker_step;
ros::Publisher marker_delete;
std::vector<int> marker_ids;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
Eigen::Vector3d odom_position;
Eigen::Quaterniond odom_quaternion;

double x_p = 0;
double y_p = 0;
double z_p = 0;
double z_high = 1.0;
double z_low = -1.0;

Eigen::Vector3d position_path(0.0, 0.0, 0.0); // Initial position
std::vector<Eigen::Vector3d> position_step_array;

ros::Subscriber sub_odom;
ros::Publisher poseArrayPub;
ros::Publisher markerarrayStepPub;
ros::Publisher odom_pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_Z_accMap = nullptr;// 使用累积地图产生的累积立方体
sensor_msgs::PointCloud2 pub_point_accMap;// 用于统一消息的头部
int marker_step_id = 0;
visualization_msgs::MarkerArray marker_step_array;

void marker(const Eigen::Vector3d &position) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
//    marker.ns = "position_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    maker_step.publish(marker);
}

/**
 *
 * @param in_cloud 累积点云
 * @param odom_position 订阅的odom的位置
 * @param length_z_clipped 裁剪的高度差
 * @return 返回的是裁剪后的z轴点云
 */
PointCloudT::Ptr
point_box_clipping_opt(PointCloudT::Ptr &in_cloud) {
    PointCloudT::Ptr filtered_ptr(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_low, z_high);
    pass.filter(*filtered_ptr);
    return filtered_ptr;
}

// 这个是外部动态调参的变量
double z_ = 0;

PointCloudT::Ptr
point_box_clipping_high(PointCloudT::Ptr &in_cloud) {
    PointCloudT::Ptr filtered_ptr(new PointCloudT);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_ + 0.0, z_ + 0.2);
    pass.filter(*filtered_ptr);
    return filtered_ptr;
}

void call_get_odom(const nav_msgs::Odometry::ConstPtr &msg) {
    z_p = msg->pose.pose.position.z;
    current_pose = msg->pose.pose;
}

double leaf_size = 0.5;
double leaf_size_filter_accMap = 0.5;
float length_z_clipped = 1;
pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);// 必须定义成全局变量
void call_point_Cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*msg, *pc);
//    auto start_f1 = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(pc);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*pc_filtered);
//    auto end_f1 = std::chrono::high_resolution_clock::now();
//    auto duration_f1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_f1-start_f1);
//    ROS_INFO_STREAM("f1_time_using: "<<duration_f1.count()<<" ms");
    *accumulated_cloud += *pc_filtered;
//    ROS_INFO_STREAM("before_f2_accumulate_size: "<<accumulated_cloud->size());

//    pcl::PointCloud<pcl::PointXYZI>::Ptr final_result(new pcl::PointCloud<pcl::PointXYZI>);
//    auto start_f2 = std::chrono::high_resolution_clock::now();
    pcl::VoxelGrid<pcl::PointXYZI> vg1;
    vg1.setInputCloud(accumulated_cloud);
    vg1.setLeafSize(leaf_size_filter_accMap, leaf_size_filter_accMap, leaf_size_filter_accMap);
    vg1.filter(*accumulated_cloud);
    auto end_f2 = std::chrono::high_resolution_clock::now();
//    auto duration_f2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_f2-start_f2);
//    ROS_INFO_STREAM("f2_time_using: "<<duration_f2.count()<<" ms");
//    ROS_INFO_STREAM("after_f2_accumulate_size: "<<accumulated_cloud->size());
// 通过z的标志位来确定
//    if (z_ == 0) {
    if (cutting_method) {
        filtered_Z_accMap = point_box_clipping_opt(accumulated_cloud);
    } else {
        filtered_Z_accMap = point_box_clipping_high(accumulated_cloud);
    }

    // ROS_INFO_STREAM("map.size11 "<<filtered_Z_accMap->size()<<"    "<<filtered_Z_accMap->width<<"  "<<filtered_Z_accMap->height);
    pub_point_accMap.header = msg->header;
}

/**
 * 发布累积地图的裁切结果
 * @param pub_filtered_points_
 */
void pub_filter_z(ros::Publisher &pub_filtered_points_) {
    if (filtered_Z_accMap == nullptr)return;
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_Z_accMap, pub_pc);
    pub_pc.header = pub_point_accMap.header;
//    pub_pc.header.frame_id = "camera_init";
    pub_filtered_points_.publish(pub_pc);
}

/**
 *
 * @param config 动态调整的参数
 * @param level 等级
 */
void callback(fast_lio::debug_paraConfig &config, uint32_t level) {
    ROS_INFO_STREAM("config.: " << config.cutting_method);
    cutting_method = config.cutting_method;

    ROS_INFO_STREAM("config.z_: " << config.z_);
    z_ = config.z_;

    ROS_INFO_STREAM("z_high: " << config.z_high);
    z_high = config.z_high;
    ROS_INFO_STREAM("z_low: " << config.z_high);
    z_low = config.z_low;

    ROS_INFO_STREAM("leaf_size: " << config.leaf_size);
    leaf_size = config.leaf_size;

    ROS_INFO_STREAM("leaf_size_filter_accMap: " << config.leaf_size_filter_accMap);
    leaf_size_filter_accMap = config.leaf_size_filter_accMap;


}

// 选点的程序
std::stack<geometry_msgs::Pose> pose_stack;//这个是保存odom当前的点
bool last_select_state = false;
int marker_id = 0;

void selectCallback(const std_msgs::Bool::ConstPtr &msg) {
    static int id = 0;
    if (msg->data == true && last_select_state == false) {
        // 当接收到/Select的数据变为true时，保存当前的Odometry数据的pose到savedPoses中
//        savedPoses.poses.push_back(current_pose);
        geometry_msgs::Pose current_goal;
        // todo:
        current_goal.position.x = position_path.x();
        current_goal.position.y = position_path.y();
        current_goal.position.z = position_path.z();
//        pose_stack.push(current_pose);//这个是保存odom当前的点
        pose_stack.push(current_goal);//这个是保存步进当前的点
        ROS_INFO_STREAM("pose_stack_size:  " << pose_stack.size());
        //  这里进行显示，maker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = ros::Time::now();
//        marker.ns = "your_namespace";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pose_stack.top().position.x;
        marker.pose.position.y = pose_stack.top().position.y;
        marker.pose.position.z = pose_stack.top().position.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 2.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_ids.push_back(marker.id);
        marker_delete.publish(marker);
//        marker_array.markers.push_back(marker);
        ROS_INFO_STREAM("marker_ids_size:" << marker_ids.size());
//        marker_array.markers.
        ROS_INFO_STREAM("save_point:" << pose_stack.top().position.x << " " << pose_stack.top().position.y << " "
                                      << pose_stack.top().position.z);
//        ROS_INFO("Saved pose: Position (%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    }
    last_select_state = msg->data;
}

std::stack<Eigen::Vector3d> pose_goal;// 保存步进点的目标点

bool pub_poses = false;
double step = 0.0;

void control_numberCallback(const std_msgs::Int32::ConstPtr &msg) {

    if (msg->data == 1) {
        step = 0.2; // Set step to 1.0m
        ROS_INFO_STREAM("step: " << step);
    }
    if (msg->data == 2) {
        step = 1.0;// Set step to 2.0m
        ROS_INFO_STREAM("step: " << step);
    }
    if (msg->data == 3) {
        step = 5.0; // Set step to 3.0m
        ROS_INFO_STREAM("step: " << step);
    }
    // 前进
    if (msg->data == 4 && step != 0) {
        position_path.x() += step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 后退
    if (msg->data == 5 && step != 0) {
        position_path.x() -= step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 向左
    if (msg->data == 6 && step != 0) {
        position_path.y() += step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 向右
    if (msg->data == 7 && step != 0) {
        position_path.y() -= step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 向上
    if (msg->data == 10 && step != 0) {
        position_path.z() += step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 向下
    if (msg->data == 11 && step != 0) {
        position_path.z() -= step;
//        addMarker(position_path);
        marker(position_path);
        ROS_INFO_STREAM("position: " << position_path.transpose());
    }
    // 表示订阅到这个删除的点
    if (msg->data == 8 && !pose_stack.empty()) {
        ROS_INFO("Delete Point: Position (%f, %f, %f)", pose_stack.top().position.x, pose_stack.top().position.y,
                 pose_stack.top().position.z);
        pose_stack.pop();
        int id_to_remove = marker_ids.back();
        marker_ids.pop_back();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_init";
//        marker.ns = "";
        marker.action = visualization_msgs::Marker::DELETE;
        marker.id = id_to_remove;
        marker_delete.publish(marker);

        ROS_INFO_STREAM("pose_stack_size:" << pose_stack.size());
        ROS_INFO_STREAM("marker_ids:" << marker_ids.size());
    }
    // 表示发布
    if (msg->data == 9 && !pose_stack.empty()) {
        pub_poses = true;
        std::stack<geometry_msgs::Pose> temp_stack;
//        geometry_msgs::PoseArray pose_temp;
        // 将原栈中的元素依次弹出并压入辅助栈
        while (!pose_stack.empty()) {
            auto element = pose_stack.top();
            pose_stack.pop();
            temp_stack.push(element);
        }
        while (!temp_stack.empty()) {
            auto element = temp_stack.top();
            temp_stack.pop();
//            geometry_msgs::PoseArray savedPoses;
            savedPoses.poses.push_back(element);
//            ROS_INFO_STREAM("pose,size NO pub"<<savedPoses.poses.size());
        }
//        poseArrayPub.publish(pose_stack);
////                    ROS_INFO_STREAM("pose,size NO pub"<<savedPoses.poses.size());
    }
}

void reset_markerArray() {
    if (!marker_ids.empty()) {
        for (int id: marker_ids) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_init";
//            marker.ns = "points";
            marker.id = id;
            marker.action = visualization_msgs::Marker::DELETE;
            marker_delete.publish(marker);
        }
        marker_ids.clear(); // Clear the vector of marker IDs
    } else {
        ROS_INFO_STREAM("no red marker need reset!");
    }
}

void reset_stack_savePoses() {
    while (!pose_stack.empty()) {
        pose_stack.pop();
    }
    savedPoses.poses.clear();
}

// 步进的marker 返回到原点。
void reset_step_marker() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_init";
    marker.header.stamp = ros::Time::now();
//    marker.ns = "position_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    maker_step.publish(marker);

}

void call_getReset(const std_msgs::Bool::ConstPtr &msg) {
    mission_reset = msg->data;
    if (msg->data) {
        position_path.setZero();
        reset_step_marker();
        reset_stack_savePoses();
        reset_markerArray();
    }
}

/**单独的程序使用laser_map的程序***/
sensor_msgs::PointCloud2 laser;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_laser(new pcl::PointCloud<pcl::PointXYZI>);// 必须定义成全局变量
void call_laser_map(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 直接进行裁切，然后将裁切之后的发布出来；
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *pc1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr x(new pcl::PointCloud<pcl::PointXYZI>);
//    if (z_ == 0) {
    if (cutting_method) {
        x = point_box_clipping_opt(pc1);
    } else {
        x = point_box_clipping_high(pc1);
    }

    cloud_laser = x;
//    *accumulated_cloud_laser += *pc1;
    laser.header = msg->header;
}

void pub_filter_laser_map(ros::Publisher &pub_filtered_points_) {
    if (cloud_laser == nullptr)return;
    sensor_msgs::PointCloud2 pub_pc1;
    pcl::toROSMsg(*cloud_laser, pub_pc1);
    pub_pc1.header = laser.header;
    pub_filtered_points_.publish(pub_pc1);
}

//ros::Publisher map_z_acc_laser;
int main(int argc, char **argv) {
    ros::init(argc, argv, "middle_node");
    ros::NodeHandle nh;

//    if(bPoint_use_laser){
    // laser_map
    ros::Subscriber laser_map_sub = nh.subscribe("/Laser_map", 100000, &call_laser_map);
    ros::Publisher map_z_acc_laser = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map_z", 100000);
//    }
    // key control
    ros::Subscriber selectSub = nh.subscribe("/Select", 100000, &selectCallback);
    ros::Subscriber KnumberSub = nh.subscribe("/control_number", 10000, &control_numberCallback);

    // marker
    marker_delete = nh.advertise<visualization_msgs::Marker>("marker_goal", 10000);
    maker_step = nh.advertise<visualization_msgs::Marker>("marker_step", 10000);

    sub_odom = nh.subscribe("/Odometry", 100000, &call_get_odom);

    // registered
    ros::Subscriber point_cloud_sub = nh.subscribe("/cloud_registered", 100000, &call_point_Cloud);
    ros::Publisher map_z_acc = nh.advertise<sensor_msgs::PointCloud2>("/map_z", 100000);

    // waypoint
    ros::Publisher poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("/waypointList", 100000);
    ros::Subscriber mission_reset_signal_Sub = nh.subscribe("/mission_reset_signal", 100000, &call_getReset);

    dynamic_reconfigure::Server<fast_lio::debug_paraConfig> server;
    dynamic_reconfigure::Server<fast_lio::debug_paraConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate rate(10);// 设置ROS程序主循环每次运行的时间至少为0.0002秒（5000Hz）
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        pub_filter_z(map_z_acc);
        if (bPoint_use_laser) pub_filter_laser_map(map_z_acc_laser);
        if (pub_poses) {
            poseArrayPub.publish(savedPoses);
            ROS_INFO_STREAM("position size: " << savedPoses.poses.size());
            ROS_INFO_STREAM("*******************pub_way_points_success!*********************");
//            ROS_INFO_STREAM("savePose"<<savedPoses.);
            pub_poses = false;
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}