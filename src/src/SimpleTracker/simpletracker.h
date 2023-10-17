#pragma once

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

class SimpleTracker
{
public:
    std::vector<SimpleTrack::Trace<T>> TraceList; // 航迹列表

public:
    SimpleTracker()
    {
        optimizer_ = new assign::HungarianOptimizer<T>();
        optimizer_->costs()->Reserve(100, 100);
    }

    ~SimpleTracker()
    {
        TraceList.clear();
        TraceList.shrink_to_fit();

        delete optimizer_;
    }

    void run(std::vector<rect_basic_struct> &detections);

private:
    size_t global_ID = 0;
    assign::HungarianOptimizer<T> *optimizer_ = nullptr;
    std::vector<std::pair<size_t, size_t>> assignments;

    void trace_predict(void);
    void trace_update(std::vector<rect_basic_struct> &detections,
                      std::vector<size_t> &trace_assignment);
    void trace_meas_assignment(std::vector<rect_basic_struct> &detections,
                               std::vector<size_t> &trace_assignment,
                               std::vector<size_t> &meas_assignment);
    void build_new_trace(std::vector<rect_basic_struct> &detections,
                         std::vector<size_t> &meas_assignment);
    std::vector<rect_basic_struct> build_2dbox_from_trace(void);
    void trace_management(void);
    std::vector<std::pair<size_t, size_t>> compute_trace_meas_distance(std::vector<rect_basic_struct> &trace_list,
                                                                       std::vector<rect_basic_struct> &meas_list,
                                                                       std::vector<std::vector<T>> &cost_mat);

    void do_nms(std::vector<rect_basic_struct> &detections);
};


static bool cmp(const std::pair<size_t, T> &a, const std::pair<size_t, T> &b);