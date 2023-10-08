#include <iostream>
#include <vector>
#include <fstream>

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
#include <pcl/visualization/cloud_viewer.h>
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

        myQuaternion.setRPY(0, 0, box.rt);
        marker.pose.orientation = tf2::toMsg(myQuaternion);

        marker.scale.x = box.l;
        marker.scale.y = box.w;
        marker.scale.z = box.h;

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_array_pub_.publish(marker_array);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SimpleTracker");
    ros::NodeHandle nh;
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bboxes", 100);

    std::cout << "It a SimpleTrack Demo" << std::endl;

    std::ifstream det_file("./src/src/SimpleTracker/1682556024.524868_det.txt");
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
                         std::atof(lineArray.at(3).c_str()),
                         std::atof(lineArray.at(3).c_str()),
                         std::atof(lineArray.at(5).c_str()),
                         std::atof(lineArray.at(6).c_str()), 0, 1.0);
        boxes.push_back(Bb);
    }

    DataIO pc_handle;
    std::string file_path = "./src/src/SimpleTracker/1682556024.724863.bin";
    PointCloudT::Ptr pc_ = pc_handle.readBinFile(file_path);

    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*pc_, laserCloudTemp);
    laserCloudTemp.header.frame_id = "SimpleTracker";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        // 发布信息
        point_pub.publish(laserCloudTemp);
        pub_boxes(boxes);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}