#include <iostream>
#include <vector>
#include <fstream>

#include "type.h"
#include "dataIO.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Dense>

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


ros::Publisher point_pub;
ros::Publisher marker_array_pub_;

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
void Proc(const visualization_msgs::MarkerArray& msg)
{
	ROS_INFO("I heard");

    // 算法开始执行
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "SimpleTracker_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("bboxes", 10, Proc); // 定义监听通道名称以及callback函数

    ros::spin();

    // 读取点云文件和检测结果
    std::vector<SimpleTrack::Trace<T>> TraceList;
    std::vector<rect_basic_struct> Meas_List;
    SimpleTrack::State<T> x;

    size_t newID = 0;
    SimpleTrack::Trace<T> new_trace(x, newID);
    TraceList.push_back(new_trace);
}