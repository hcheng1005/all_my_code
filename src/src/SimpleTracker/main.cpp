#include <iostream>
#include <vector>
#include <fstream>
#include <filesystem>
#include <algorithm>

#include "type.h"
#include "dataIO.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <Eigen/Dense>

ros::Publisher point_pub;
ros::Publisher marker_array_pub_;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
static void pub_boxes(const std::vector<Bndbox> boxes)
{
    visualization_msgs::MarkerArray marker_array;
    tf2::Quaternion myQuaternion;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t id = 0;
    for (auto box : boxes)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "SimpleTracker";
        marker.header.stamp = ros::Time::now();

        marker.id = id;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = box.x;
        marker.pose.position.y = box.y;
        marker.pose.position.z = box.z;

        // std::cout << "box.rt: " << box.rt / M_PI * 180.0 << std::endl;

        myQuaternion.setRPY(0, 0, box.rt);
        marker.pose.orientation = tf2::toMsg(myQuaternion);

        marker.scale.x = box.l;
        marker.scale.y = box.w;
        marker.scale.z = box.h;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.color.a = 0.4;

        marker.lifetime = ros::Duration(0.1);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_array_pub_.publish(marker_array);
}

/**
 * @names: 
 * @description: Briefly describe the function of your function
 * @param {string} &file_path
 * @return {*}
 */
std::vector<Bndbox> read_detection_from_file(std::string &file_path)
{
    std::ifstream det_file(file_path);
    std::string lineStr;
    std::vector<std::string> lineArray;
    std::string str_line;

    std::vector<Bndbox> boxes;
    while (std::getline(det_file, lineStr))
    {
        // lineArray.push_back(lineStr);
        std::stringstream ss(lineStr);
        lineArray.clear();
        while (getline(ss, str_line, ','))
        {
            lineArray.push_back(str_line);
        }

        auto Bb = Bndbox(std::atof(lineArray.at(0).c_str()),
                         std::atof(lineArray.at(1).c_str()),
                         std::atof(lineArray.at(2).c_str()),
                         std::atof(lineArray.at(4).c_str()),
                         std::atof(lineArray.at(3).c_str()),
                         std::atof(lineArray.at(5).c_str()),
                         std::atof(lineArray.at(6).c_str()), 0, 1.0);
        boxes.push_back(Bb);
    }

    return boxes;
}

/**
 * @names: 
 * @description: Briefly describe the function of your function
 * @param {string} &file_path
 * @return {*}
 */
sensor_msgs::PointCloud2 read_pointcloud_from_file(std::string &file_path)
{
    DataIO pc_handle;

    PointCloudT::Ptr pc_ = pc_handle.readBinFile(file_path); // 读取点云文件

    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*pc_, laserCloudTemp);
    laserCloudTemp.header.frame_id = "SimpleTracker";

    return laserCloudTemp;
}

/**
 * @names: 
 * @description: Briefly describe the function of your function
 * @return {*}
 */
bool sort_file_by_name(std::string&a, std::string&b)
{
    double aa = std::atof(a.c_str());
    double bb = std::atof(b.c_str());

    return (aa < bb);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
using std::filesystem::directory_iterator; // SUPPORTED FROM C++17
using std::filesystem::path;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "SimpleTracker");
    ros::NodeHandle nh;
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bboxes", 100);

    std::cout << "It a SimpleTrack Demo" << std::endl;

    // 遍历点云文件
    ros::Rate loop_rate(20);
    DataIO pc_handle;
    std::string folder_path_pc = "/home/zdhjs-05/myGitHubCode/all_my_code/src/src/SimpleTracker/pc/";
    std::string folder_path_det = "/home/zdhjs-05/myGitHubCode/all_my_code/src/src/SimpleTracker/det/";
    std::string pc_path, det_path;
    std::vector<std::string> file_list;
    for (auto &v : directory_iterator(folder_path_pc))
    {
       std::string sub_file = v.path().stem().string();
       file_list.push_back(sub_file);
    }

    std::sort(file_list.begin(), file_list.end(), sort_file_by_name);

    for (auto &file_ : file_list)
    {
        pc_path = folder_path_pc + file_ + ".bin";
        det_path = folder_path_det + file_ + "_det.txt";

        auto laserCloud = read_pointcloud_from_file(pc_path);
        auto boxes = read_detection_from_file(det_path);
    
        point_pub.publish(laserCloud); // 发布点云信息
        pub_boxes(boxes);

        loop_rate.sleep();
    }

    ros::spin();

    return 1;
}