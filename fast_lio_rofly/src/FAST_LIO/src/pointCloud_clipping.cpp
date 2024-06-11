/**
 * the code implemented in C language style
 * Created by pf
 */
#include <iostream>
#include <Eigen/Core>
#include <deque>
#include <fstream>
#include <cmath>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// 投影的图像大小
int IMG_WIDTH = 1000;
int IMG_HEIGHT = 1000;
double lidar_obstacle_radius = 4;
// 点云点的格式
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
// 使用使用累积的地图，0使用累积产生地图，1使用滑动窗口的地图
const bool isDenseBox = 1;
// 旋转点还是旋转图像 1使用的是根据yaw角旋转点，0根据yaw旋转图像
const bool isRotationLidarPoint_inbox = 1;
// 对应word的位姿的box，外面会给这个参数
double length_x_clipped = 0;
double length_y_clipped = 0;
double length_z_clipped = 0;
// 对应word的位姿的中心点
double x = 0;
double y = 0;
double z = 0;

Eigen::Vector3d euler(0, 0, 0);
Eigen::Vector3d marker_pose(0,0,0);

ros::Subscriber sub_point_cloud_;
ros::Subscriber sub_odom_boxMapping;
ros::Subscriber sub_ladar_map_feats;
ros::Publisher  odom_pub;
ros::Subscriber sub_marker;

// 将box的点云发出来
ros::Publisher pub_filtered_points_;
ros::Publisher pub_box_image;

pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_accMap = nullptr;// 累积地图产生的一帧点云
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_box_accMap = nullptr;// 使用累积地图产生的累积立方体
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_box_accWindow = nullptr;// 使用累积地图产生的累积立方体
pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_box_1 = nullptr;
sensor_msgs::PointCloud2 pub_point_accMap;// 用于统一消息的头部
sensor_msgs::PointCloud2 pub_point_accWindow;// 用于统一消息的头部
pcl::PointCloud<pcl::PointXYZI>::Ptr box_window_accWindow(new pcl::PointCloud<pcl::PointXYZI>);// 滑动窗口裁切的结果
/**
 * @param in_cloud_ptr 订阅acc_Map对应的累积地图处理
 * 这里是修改了fastlio输出的累积地图，默认是不使用
 */
void call_point_box_clipping(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
    current_point_accMap = current_pc_ptr;
    // 对应的下采样的代码
    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // vg.setInputCloud(current_pc_ptr);
    // float data_x = 1.0f;
    // vg.setLeafSize(data_x, data_x, data_x);
    // vg.filter(*filtered_pc_ptr);

    PointT minValues, maxValues;
    pcl::getMinMax3D(*current_pc_ptr, minValues, maxValues);//得到输入点云x，y，z三个轴上的最小值最大值

    // double y_middle = (minValues.y + maxValues.y) * 0.5;
    // double x_middle = (minValues.y + maxValues.y) * 0.5;
    // double z_middle = (minValues.z + maxValues.z) * 0.5;
    //  ROS_INFO_STREAM("x,y,z"<<x<<"\t"<<y<<"\t"<<z<<"\n");

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(current_pc_ptr);
    pass.setFilterFieldName("z");
    // pass.setFilterLimits(z_middle - length_z_clipped * 0.9, z_middle + length_z_clipped * 0.1);
    pass.setFilterLimits(z - length_z_clipped * 0.5, z + length_z_clipped * 0.5);
    //    pass.setNegative(false);
    pass.filter(*current_pc_ptr);

    pass.setInputCloud(current_pc_ptr);
    pass.setFilterFieldName("y");
    // pass.setFilterLimits(y_middle - length_y_clipped * 0.5, y_middle + length_y_clipped * 0.5);
    pass.setFilterLimits(y - length_y_clipped * 0.5, y + length_y_clipped * 0.5);

    //    pass.setNegative(false);
    pass.filter(*current_pc_ptr);

    pass.setInputCloud(current_pc_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x - length_x_clipped * 0.5, x + length_x_clipped * 0.5);
    // pass.setFilterLimits(x_middle - length_x_clipped * 0.5, x_middle + length_x_clipped * 0.5);
    //    pass.setNegative(false);
    pass.filter(*filtered_pc_ptr);
    filtered_box_accMap = filtered_pc_ptr;
    // sensor_msgs::PointCloud2 pub_pc;
    // pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
    // pub_pc.header = in_cloud_ptr->header;
    // pub_pc1.header = pub_pc.header;
    // pub_filtered_points_.publish(pub_pc);
    pub_point_accMap.header = in_cloud_ptr->header;
}

/**
 * 裁剪一帧点云
 * @param in_cloud  滑动窗口对应的一帧点云，累积的点云大小可以自己确定，基于点云的个数
 * @return 返回的是滑动窗口中的累积点云
 */
PointCloudT::Ptr point_box_clipping_opt(pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud) {

    PointCloudT::Ptr current_ptr(new PointCloudT);
    PointCloudT::Ptr filtered_ptr(new PointCloudT);

    pcl::PassThrough<PointT> pass;
    // 依次裁剪，z,y,x
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    // pass.setFilterLimits(z - length_z_clipped * 0.5, z + length_z_clipped * 0.5);
    pass.setFilterLimits(0, 0.3);
    pass.filter(*current_ptr);

    pass.setInputCloud(current_ptr);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y - length_y_clipped * 0.5, y + length_y_clipped * 0.5);
    pass.filter(*current_ptr);

    pass.setInputCloud(current_ptr);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x - length_x_clipped * 0.5, x + length_x_clipped * 0.5);
    pass.filter(*filtered_ptr);
    filtered_box_accWindow = filtered_ptr;

    return filtered_ptr;
}

/**
 * 发布累积地图的裁切结果
 * @param pub_filtered_points_
 */
void pub_filter_box(ros::Publisher &pub_filtered_points_) {
    // filtered_pc_box = filtered_pc_ptr;
    if (filtered_box_accMap == nullptr)return;
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_box_accMap, pub_pc);
    pub_pc.header = pub_point_accMap.header;
    pub_filtered_points_.publish(pub_pc);
}

/**
 * 如果是累积多地图的裁切结果是：filtered_box_accMap
 * 如果是滑动窗口的裁切结果结果是：filtered_box_accWindow
 * 更换这个即可
 * @param pub_box_image
 */
/**
 * 绘制agent的梭形结构
 */
cv::Point2f vertices_points[4];

void draw_agent(cv::Mat &image, cv::Point &point) {
    // 定义四边形的参数
    int size = 50;  // 四边形的大小
    // 计算四边形的顶点坐标
    cv::Point top(point.x, point.y + size / 2);
    cv::Point bottom_left(point.x - size / 2, point.y - size / 2);
    cv::Point bottom_right(point.x + size / 2, point.y - size / 2);
    // 定义四边形的顶点数组
    std::vector<cv::Point> triangle_points = {top, bottom_left, point, bottom_right};
    vertices_points[0] = triangle_points[0];
    vertices_points[1] = triangle_points[1];
    vertices_points[2] = triangle_points[2];
    vertices_points[3] = triangle_points[3];
    // 绘制四边形的轮廓
    cv::line(image, top, bottom_left, cv::Scalar(255, 0, 0), 2);
    cv::line(image, bottom_right, top, cv::Scalar(255, 0, 0), 2);
    cv::line(image, bottom_left, point, cv::Scalar(255, 0, 0), 2);
    cv::line(image, bottom_right, point, cv::Scalar(255, 0, 0), 2);
    // 填充三角形内部，对应四边形的颜色，这里是
    cv::fillPoly(image, std::vector<std::vector<cv::Point>>{triangle_points}, cv::Scalar(255, 0, 0));
}

/// @brief 根据自身的yaw角度改变agent的梭形的角度，也就是yaw角
/// @param image 投影的图像
/// @param yaw 
/// @param center pose的位置
/// @param vertices 图标的四个点
double yaw_body = 0;

void body_yaw(cv::Mat image, double yaw, const cv::Point2f &center, const cv::Point2f vertices[4]) {
    // 计算旋转后的顶点位置
    double radian = -yaw;
    cv::Point2f rotatedVertices[4];
    for (int i = 0; i < 4; ++i) {
        double agent_x = vertices[i].x - center.x;
        double agent_y = vertices[i].y - center.y;
        rotatedVertices[i].x = agent_x * cos(radian) - agent_y * sin(radian) + center.x;
        rotatedVertices[i].y = agent_x * sin(radian) + agent_y * cos(radian) + center.y;
    }
    // 绘制旋转后的四边形
    cv::line(image, rotatedVertices[0], rotatedVertices[1], cv::Scalar(0, 0, 255), 2);
    cv::line(image, rotatedVertices[1], rotatedVertices[2], cv::Scalar(0, 0, 255), 2);
    cv::line(image, rotatedVertices[2], rotatedVertices[3], cv::Scalar(0, 0, 255), 2);
    cv::line(image, rotatedVertices[3], rotatedVertices[0], cv::Scalar(0, 0, 255), 2);
//    cv::fillPoly(image, std::vector<std::vector<cv::Point>>{triangle_points}, cv::Scalar(255, 0, 0));
}

void point_yaw(double yaw_radians, const cv::Point2f &center, cv::Point2f &point_rotation) {
    // double radians = yaw * M_PI / 180.0;
    // 计算旋转矩阵的元素
    double cosAngle = cos(yaw_radians);
    double sinAngle = sin(yaw_radians);
    // 将点坐标平移到旋转中心
    double translatedX = point_rotation.x - center.x;
    double translatedY = point_rotation.y - center.y;
    // 计算旋转后的点坐标
    double rotatedX = translatedX * cosAngle - translatedY * sinAngle;
    double rotatedY = translatedX * sinAngle + translatedY * cosAngle;

    //  将点坐标平移回原来位置
    rotatedX += center.x;
    rotatedY += center.y;
    // 返回自身
    point_rotation.x = rotatedX;
    point_rotation.y = rotatedY;
}

void pub_image(ros::Publisher &pub_box_image) {
    if (filtered_box_accMap == nullptr && filtered_box_accWindow == nullptr) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr box_point(new pcl::PointCloud<pcl::PointXYZI>);
    // 选择采用的地图
    if (!isDenseBox) {
        box_point = filtered_box_accMap;
    } else {
        box_point = filtered_box_accWindow;
    }
    // 确定地图的原点
    float x_0 = x - length_x_clipped * 0.5;
    float y_0 = y - length_y_clipped * 0.5;

    // 缩放因子
    float scale_x = static_cast<float>(IMG_WIDTH - 1) / static_cast<float>(length_x_clipped);
    float scale_y = static_cast<float>(IMG_HEIGHT - 1) / static_cast<float>(length_y_clipped);
    // 绘制图像
    cv::Mat image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));


    // 确定中心
    int x_center = static_cast<int>((x - x_0) * scale_x);
    int y_center = static_cast<int>((y - y_0) * scale_y);
    cv::Point center(x_center, y_center);

    for (const auto &point: box_point->points) {
        int u = static_cast<int>((point.x - x_0) * scale_x);
        int v = static_cast<int>((point.y - y_0) * scale_y);

        if (isRotationLidarPoint_inbox) {
            cv::Point2f point_uv_origin(u, v);
            point_yaw(-yaw_body, center, point_uv_origin);
            cv::circle(image, cv::Point(point_uv_origin.y, point_uv_origin.x), lidar_obstacle_radius,
                       cv::Scalar(0, 0, 255), cv::FILLED);
        } else {
            // 在图像上绘制点
            cv::circle(image, cv::Point(v, u), lidar_obstacle_radius, cv::Scalar(0, 0, 255), cv::FILLED);
        }
    }

    // Marker pose:
    int marker_u = static_cast<int>((marker_pose[0] - x_0) * scale_x);
    int marker_v = static_cast<int>((marker_pose[1] - y_0) * scale_y);
    cv::Point2f marker_uv_origin(marker_u, marker_v);
    point_yaw(-yaw_body, center, marker_uv_origin);
    cv::circle(image, cv::Point(marker_uv_origin.y, marker_uv_origin.x), 12.0, cv::Scalar(0, 255, 0), cv::FILLED);
    
    if (!isRotationLidarPoint_inbox) {
        // 将弧度制转换为角度制
        cv::Point2f point1(IMG_HEIGHT * 0.5, IMG_WIDTH * 0.5);
        // 获得旋转矩阵
        cv::Mat rotation = cv::getRotationMatrix2D(point1, -fmod(yaw_body * 180.0 / M_PI, 360.0), 1.0);
        // ROS_INFO_STREAM("body_yaw"<<yaw_body*180/M_1_PI<<"\n");
        // ROS_INFO_STREAM("body_yaw"<<"\t"<<fmod(yaw_body * 180.0 / M_PI, 360.0)<<"\n");
        cv::warpAffine(image, image, rotation, image.size());// 旋转图像
    }
//    ROS_INFO_STREAM("filter_x" << "\t" << x_center << "filter_y: " << y_center << "\n");

    /***中心旋转处理*/
    // 绘制中心
    draw_agent(image, center);
    // 跟随yaw进行旋转，这个不需要，所以注释
    // body_yaw(image, yaw_body, center, vertices_points);
    // cv::rotate(image, image, cv::ROTATE_180);
    cv::flip(image, image, -1);

    /*******************************添加文字***************************/
    std::string mapSize = "map size: "+std::to_string(static_cast<int>(length_x_clipped))+"x"+std::to_string(static_cast<int>(length_y_clipped))+"m";
    cv::putText(image, mapSize, cv::Point(40, 40), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
    if (image.empty()) std::cout << "image is empty" << std::endl;
    // filtered_pc_box.reset();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    if (!isDenseBox) {
        msg->header = pub_point_accMap.header;
    } else {
        msg->header = pub_point_accWindow.header;
    }
    pub_box_image.publish(msg);
}
void call_get_marker(const visualization_msgs::Marker::ConstPtr& msg){
    marker_pose[0] = msg->pose.position.x;
    marker_pose[1] = msg->pose.position.y;
    marker_pose[2] = msg->pose.position.z;
    ROS_WARN("marker_pose = %.2f, %.2f, %.2f",marker_pose[0], marker_pose[1], marker_pose[2]);
}
void call_get_odom(const nav_msgs::Odometry::ConstPtr &msg) {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    // 使用ros中的四元数转欧拉角
    nav_msgs::Odometry new_odom = *msg;
    tf2::Quaternion quat;
    tf2::fromMsg(new_odom.pose.pose.orientation, quat);
    double new_roll = 0, new_pitch = 0, new_yaw = 0;
    tf2::Matrix3x3(quat).getRPY(new_roll, new_pitch, new_yaw);
//    提取这个yaw角
    yaw_body = new_yaw;
    // 更改欧拉角并发布，这里直接设置为0
    new_roll = 0.0;
    new_pitch = 0.0;
    new_yaw = 0.0;
    tf2::Quaternion new_quat;
    new_quat.setRPY(new_roll, new_pitch, new_yaw);
    new_odom.pose.pose.orientation = tf2::toMsg(new_quat);
    new_odom.header = msg->header;
// 使用eigen中的四元数转欧拉角
//    Eigen::Quaterniond q(new_odom.pose.pose.orientation.w, new_odom.pose.pose.orientation.x, new_odom.pose.pose.orientation.y,
//                        new_odom.pose.pose.orientation.z);
//    euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//    ROS_INFO_STREAM("euler,x,y,z \t" << euler[2] << "\t" << euler[1] << "\t" << euler[0] << "\n");
    odom_pub.publish(new_odom);
}

// 滑动窗口的大小
std::deque<sensor_msgs::PointCloud2::ConstPtr> window_buffer;
size_t max_window_size = 50;

/***
 * @param msg：订阅一帧的点云，进行滑动窗口积累数据，对应的点云大小可控
 */
void call_point_slidingwindow_Cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 添加点云数据到滑动窗口缓冲区
    window_buffer.push_back(msg);
    // 如果滑动窗口大小超过最大限制，则移除最旧的数据
    if (window_buffer.size() > max_window_size) {
        window_buffer.pop_front();
    }
//    ROS_INFO_STREAM("buffer size:" << window_buffer.size() << "\n");
    // 累加窗口中的点云数据，需要初始化，否则运行时会报错
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &pc_msg: window_buffer) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*pc_msg, *pc);
        *accumulated_cloud += *pc;
    }
    // 累积之后进行裁切
    box_window_accWindow = point_box_clipping_opt(accumulated_cloud);
    pub_point_accWindow.header = msg->header;
    // accumulated_cloud = nullptr;
}

// 发布这个滑动窗口的结果，发布累加后的点云数据
void pub_window_box(ros::Publisher &window_pub) {

    if (window_buffer.size() >= 50) {
        sensor_msgs::PointCloud2 accumulated_msg;
        pcl::toROSMsg(*box_window_accWindow, accumulated_msg);
        if (!isDenseBox) {
            accumulated_msg.header = pub_point_accMap.header;
        } else {
            accumulated_msg.header = pub_point_accWindow.header;//使用自己的header
        }
        // accumulated_msg.header = window_buffer.back()->header;  // 使用最新点云消息的头部信息
        window_pub.publish(accumulated_msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "middle_node");
    ros::NodeHandle nh;
    nh.param<double>("length_x_clipped", length_x_clipped, 5.0);//平面点
    nh.param<double>("length_y_clipped", length_y_clipped, 5.0);//平面点
    nh.param<double>("length_z_clipped", length_z_clipped, 0.3);//平面点
    nh.param<double>("lidar_obstacle_radius", lidar_obstacle_radius, 4.0);//平面点
    nh.param<int>("IMG_WIDTH", IMG_WIDTH, 1000);//平面点
    nh.param<int>("IMG_HEIGHT", IMG_WIDTH, 1000);//平面点

    // sub_ladar_map_feats = nh.subscribe("/cloud_registered",100000, &point_box_clipping);
    // sub_ladar_map_feats = nh.subscribe("/Laser_map",100000, &point_box_clipping);
    sub_ladar_map_feats = nh.subscribe("/Laser_ACCmap", 100000, &call_point_box_clipping);
    // sub_ladar_map_feats = nh.subscribe("/cloud_registered", 100000, &call_point_box_clipping);
    sub_odom_boxMapping = nh.subscribe("/Odometry", 100000, &call_get_odom);
    sub_marker = nh.subscribe("/preview_goal_marker", 10, &call_get_marker);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/new_odom", 100000);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100000);
    pub_box_image = nh.advertise<sensor_msgs::Image>("/box_image", 100000);

    ros::Subscriber point_cloud_sub = nh.subscribe("/cloud_registered", 100000, &call_point_slidingwindow_Cloud);
    ros::Publisher window_pub = nh.advertise<sensor_msgs::PointCloud2>("/sliding_window", 100000);
    std::ofstream outputFile("/home/pf/ws_livox/src/FAST_LIO/pf_output.csv");// 用于调试输出

    ros::Rate rate(5000);// 设置ROS程序主循环每次运行的时间至少为0.0002秒（5000Hz）
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        if (!isDenseBox) {
            pub_filter_box(pub_filtered_points_);
            pub_image(pub_box_image);
        } else {
            pub_image(pub_box_image);
            pub_window_box(window_pub);
//            outputFile << img_u << "," << img_v << std::endl;
        }
        status = ros::ok();
        rate.sleep();
    }
//    outputFile.close();
    // ros::spin();
    return 0;
}