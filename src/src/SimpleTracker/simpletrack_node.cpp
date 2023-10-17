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
// Eigen
#include <Eigen/Dense>

#include "simpletracker.h"

SimpleTracker myTracker;

ros::Publisher point_pub;
ros::Publisher marker_array_pub_;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
static void pub_trace_boxes(void)
{

    std::cout << "trace number: "<< myTracker.TraceList.size() << std::endl;

    visualization_msgs::MarkerArray marker_array;
    tf2::Quaternion myQuaternion;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t id = 0;
    for (auto trace : myTracker.TraceList)
    {
        if (!trace.valid)
        {
            continue;
        }
        auto x_state = trace.filter.getState();

        visualization_msgs::Marker marker;
        marker.header.frame_id = "nuscenes";
        marker.header.stamp = ros::Time::now();

        marker.id = trace.ID;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x_state.y();
        marker.pose.position.y = x_state.x();
        marker.pose.position.z = x_state.z();

        // std::cout << "box.rt: " << box.rt / M_PI * 180.0 << std::endl;

        myQuaternion.setRPY(0, 0, x_state.theta());
        marker.pose.orientation = tf2::toMsg(myQuaternion);

        marker.scale.x = x_state.wid();
        marker.scale.y = x_state.len();
        marker.scale.z = x_state.height();

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.color.a = 0.6;

        marker.lifetime = ros::Duration(1.5);
        marker_array.markers.push_back(marker);
        id++;
    }

    marker_array_pub_.publish(marker_array);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MarkerArray&} msg
 * @return {*}
 */
void Proc(const visualization_msgs::MarkerArray &msg)
{
    ROS_INFO("Rec A New Msg");

    // 算法开始执行
    std::vector<rect_basic_struct> dets;
    for (auto &det : msg.markers)
    {
        rect_basic_struct box;
        box.center_pos[0] = det.pose.position.y; // 纵向
        box.center_pos[1] = det.pose.position.x; // 横向
        box.center_pos[2] = det.pose.position.z;

        box.box_len = det.scale.y;
        box.box_wid = det.scale.x;
        box.box_height = det.scale.z;

        auto q = det.pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        box.heading = std::atan2(siny_cosp, cosy_cosp);

        box.score =  std::atof(det.text.c_str());

        // std::cout << det.text.c_str()  << std::endl;
        // std::cout << "theta " << box.heading / M_PI * 180.0 << std::endl;
        // std::cout << "[x,y,z]:" << det.pose.position.x << ", "<< det.pose.position.y << ", "<< det.pose.position.z << std::endl;

        dets.push_back(box);
    }

    // 跟踪算法入口
    myTracker.run(dets);

    pub_trace_boxes();
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {int} argc
 * @param {char} *
 * @return {*}
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "SimpleTracker_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("visualization_marker", 10, Proc); // 定义监听通道名称以及callback函数
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("trace", 100);

    ros::spin();
}