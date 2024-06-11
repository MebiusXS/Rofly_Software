/*******************************************************
 * Author: Wang PengFei (pfw2024@outlook.com)
 * Created by pf on 2024/5/24.
 *
 * The main function of the code is to accumulate maps for FastLio and prepare for EGO planning
 *******************************************************/
#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace MapEgo {
    double leaf_size = 0.1;
    double leaf_size_filter_accMap = 0.1;
    double box_x_length = 20;
    double box_y_length = 20;
    double box_z_length = 20;
    Eigen::Vector3d Odom_position(0, 0, 0);
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_box_accMap = nullptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    PointCloudT::Ptr point_box_clipping(const PointCloudT::Ptr &in_cloud, const Eigen::Vector3d &position) {
        PointCloudT::Ptr filtered_ptr(new PointCloudT);
        ROS_INFO_STREAM_ONCE("*-*-*-*-*-*para_box:" << box_x_length << " " << box_y_length << " " << box_z_length);
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(in_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(static_cast<float>(position.z() - box_z_length * 0.5),
                             static_cast<float>(position.z() + box_z_length * 0.5));
        pass.filter(*filtered_ptr);

        pass.setInputCloud(filtered_ptr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(static_cast<float>(position.x() - box_x_length * 0.5),
                             static_cast<float>(position.x() + box_x_length * 0.5));
        pass.filter(*filtered_ptr);

        pass.setInputCloud(filtered_ptr);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(static_cast<float>(position.y() - box_y_length * 0.5),
                             static_cast<float>(position.z() + box_y_length * 0.5));
        pass.filter(*filtered_ptr);

        return filtered_ptr;
    }

    void call_get_odom(const nav_msgs::Odometry::ConstPtr &msg) {
        Odom_position.x() = msg->pose.pose.position.x;
        Odom_position.y() = msg->pose.pose.position.y;
        Odom_position.z() = msg->pose.pose.position.z;
    }

    void call_point_Cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *pc);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(pc);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*pc_filtered);
        *accumulated_cloud += *pc_filtered;
        pcl::VoxelGrid<pcl::PointXYZI> vg1;
        vg1.setInputCloud(accumulated_cloud);
        vg1.setLeafSize(leaf_size_filter_accMap, leaf_size_filter_accMap, leaf_size_filter_accMap);
        vg1.filter(*accumulated_cloud);

        filtered_box_accMap = point_box_clipping(accumulated_cloud, Odom_position);
    }

    void pub_filter_box(ros::Publisher &pub_filtered_points_) {
        if (filtered_box_accMap == nullptr)return;
        sensor_msgs::PointCloud2 pub_pc;
        pcl::toROSMsg(*filtered_box_accMap, pub_pc);
        pub_pc.header.frame_id = "camera_init";
        pub_pc.header.stamp = ros::Time::now();
        pub_filtered_points_.publish(pub_pc);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "MapEgo");
    ros::NodeHandle nh;

    nh.param<double>("leaf_size", MapEgo::leaf_size, 0.1);
    nh.param<double>("leaf_size_filter_accMap", MapEgo::leaf_size_filter_accMap, 0.1);
    nh.param<double>("box_x_length", MapEgo::box_x_length, 20.0);
    nh.param<double>("box_y_length", MapEgo::box_y_length, 20.0);
    nh.param<double>("box_z_length", MapEgo::box_z_length, 20.0);

    ROS_INFO_STREAM("Map for EGO has been published!");

    ros::Subscriber point_cloud_sub = nh.subscribe("/cloud_registered", 100000, &MapEgo::call_point_Cloud);
    ros::Subscriber sub_odom = nh.subscribe("/Odometry", 100000, &MapEgo::call_get_odom);
    ros::Publisher map_acc_box = nh.advertise<sensor_msgs::PointCloud2>("/MapEgo", 100000);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        MapEgo::pub_filter_box(map_acc_box);
        rate.sleep();
    }
    return 0;
}