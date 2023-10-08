#include <iostream>
#include <vector>
#include <fstream>
#include "dataIO.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

#include "PositionMeasurementModel.hpp"
#include "SystemModel.hpp"
#include "TraceManage.hpp"
#include <distance/iou.h>
#include <assignment/hungarian_optimizer.h>

using namespace TrackerDemo;
typedef float T;

// Some type shortcuts
typedef SimpleTrack::State<T> State;
typedef SimpleTrack::Control<T> Control;
typedef SimpleTrack::SystemModel_CAWithShape<T> SystemModel;
typedef SimpleTrack::PositionMeasurementModel_3DBBox<T> MeasModel;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
void trace_predict(std::vector<SimpleTrack::Trace<T>> &TraceList)
{
    for (auto &sub_trace : TraceList)
    {
        sub_trace.Predict();
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
void trace_update(std::vector<SimpleTrack::Trace<T>> &TraceList)
{
    SimpleTrack::PositionMeasurement_3DBBox<T> meas_;
    for (auto &sub_trace : TraceList)
    {
        sub_trace.Update(meas_);
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
std::vector<rect_basic_struct> build_2dbox_from_trace(const std::vector<SimpleTrack::Trace<T>> &TraceList)
{
    std::vector<rect_basic_struct> trace_box_list;
    SimpleTrack::State<T> x;
    for (auto &sub_trace : TraceList)
    {
        rect_basic_struct new_trace_box;
        x = sub_trace.filter.getState();
        new_trace_box.center_pos[0] = x.x();
        new_trace_box.center_pos[1] = x.y();
        new_trace_box.center_pos[2] = x.z();
        new_trace_box.box_len = x.len();
        new_trace_box.box_wid = x.wid();
        new_trace_box.box_height = x.height();

        trace_box_list.push_back(new_trace_box);
    }

    return trace_box_list;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
std::vector<std::pair<size_t, size_t>> compute_trace_meas_distance(std::vector<rect_basic_struct> &trace_list,
                                                                   std::vector<rect_basic_struct> &meas_list)
{
    assign::HungarianOptimizer<T> optimizer_;
    optimizer_.costs()->Reserve(100, 100);
    std::vector<std::pair<size_t, size_t>> assignments;

    // 构造代价矩阵（使用2DIOU作为距离度量）
    for (size_t i = 0; i < meas_list.size(); i++)
    {
        for (size_t j = 0; j < trace_list.size(); j++)
        {
            T iou_ = IOU_2D(trace_list.at(i), meas_list.at(j));
            (*optimizer_.costs())(i, j) = iou_;
        }
    }

    optimizer_.costs()->Resize(meas_list.size(), trace_list.size());
    optimizer_.Minimize(&assignments);
    optimizer_.PrintMatrix();

    return assignments;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
ros::Publisher point_pub;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_cluster");
    ros::NodeHandle nh;
    point_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    // ros::spin();

    std::cout << "It a SimpleTrack Demo" << std::endl;

    // 读取点云文件和检测结果
    std::vector<SimpleTrack::Trace<T>> TraceList;
    std::vector<rect_basic_struct> Meas_List;
    SimpleTrack::State<T> x;

    size_t newID = 0;
    SimpleTrack::Trace<T> new_trace(x, newID);
    TraceList.push_back(new_trace);

    // std::cout << TraceList.size() << std::endl;

    // std::ifstream det_file("/home/zdhjs-05/myGitHubCode/all_my_code/src/src/SimpleTracker/1682556024.524868_det.txt");
    // std::string lineStr;
    // std::vector<std::string> lineArray;
    // std::string str_line;
    // while (std::getline(det_file, lineStr))
    // {
    //     // lineArray.push_back(lineStr);
    //     std::stringstream ss(lineStr);
    //     lineArray.clear();
    //     while (getline(ss, str_line, ','))
    //     {
    //         lineArray.push_back(str_line);
    //     }

    //     rect_basic_struct new_det;
    //     new_det.center_pos[0] = std::atof(lineArray.at(0));
    //     new_det.center_pos[1] = std::atof(lineArray.at(1));
    //     new_det.center_pos[2] = std::atof(lineArray.at(2));
    //     new_det.box_len = std::atof(lineArray.at(3));
    //     new_det.box_wid = std::atof(lineArray.at(4));
    //     new_det.box_height = std::atof(lineArray.at(5));
    //     new_det.heading = std::atof(lineArray.at(6));
    //     Meas_List.push_back(new_det);
    // }

    DataIO pc_handle;
    std::string file_path = "/home/zdhjs-05/myGitHubCode/all_my_code/src/src/SimpleTracker/1682556024.724863.bin";
    PointCloudT::Ptr pc_ = pc_handle.readBinFile(file_path);

    sensor_msgs::PointCloud2 laserCloudTemp;
    pcl::toROSMsg(*pc_, laserCloudTemp);
    laserCloudTemp.header.frame_id = "point_cloud";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        //发布信息
        point_pub.publish(laserCloudTemp);
        ros::spinOnce();
        loop_rate.sleep();

        std::cout << "111" << std::endl;
    }

    std::cout << "222" << std::endl;

    return 1;
}