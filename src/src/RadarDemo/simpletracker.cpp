#include <iostream>
#include <vector>
#include <fstream>

#include "simpletracker.h"

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
void SimpleTracker::trace_predict(void)
{
    // std::cout << "----- trace_predict ----- " << std::endl;
    for (auto &sub_trace : TraceList)
    {
        // std::cout << "old x info: " << sub_trace.filter.getState().x() << ", " << sub_trace.filter.getState().y() << ", " << sub_trace.filter.getState().z() << ", "
        //           << sub_trace.filter.getState().vx() << ", " << sub_trace.filter.getState().vy() << ", " << sub_trace.filter.getState().vz() << ", "
        //           << sub_trace.filter.getState().ax() << ", " << sub_trace.filter.getState().ay() << ", " << sub_trace.filter.getState().az() << ", " << std::endl;
        
        sub_trace.Predict();

        // std::cout << "new x info: " << sub_trace.filter.getState().x() << ", " << sub_trace.filter.getState().y() << ", " << sub_trace.filter.getState().z() << ", "
        //           << sub_trace.filter.getState().vx() << ", " << sub_trace.filter.getState().vy() << ", " << sub_trace.filter.getState().vz() << ", "
        //           << sub_trace.filter.getState().ax() << ", " << sub_trace.filter.getState().ay() << ", " << sub_trace.filter.getState().az() << ", " << std::endl;
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<rect_basic_struct>} &detections
 * @return {*}
 */
void SimpleTracker::trace_meas_assignment(std::vector<rect_basic_struct> &detections,
                                          std::vector<size_t> &trace_assignment,
                                          std::vector<size_t> &meas_assignment)
{
    std::vector<rect_basic_struct> traces;

    trace_assignment.resize(TraceList.size(), 255);
    meas_assignment.resize(detections.size(), 1);

    traces = build_2dbox_from_trace();

    // Build CostMatrix by Computing 2DIOU Between Traces And Measments
    assignments = compute_trace_meas_distance(traces, detections);

    for (const auto &sub_pair : assignments)
    {
        trace_assignment.at(sub_pair.second) = sub_pair.first;
        meas_assignment.at(sub_pair.first) = 0; // means this measment would not to be a new "trace"
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
std::vector<std::pair<size_t, size_t>> SimpleTracker::compute_trace_meas_distance(std::vector<rect_basic_struct> &trace_list,
                                                                                  std::vector<rect_basic_struct> &meas_list)
{
    // std::cout << "Do compute_trace_meas_distance" << std::endl;
    std::vector<std::pair<size_t, size_t>> assignment;
    // std::vector<std::pair<size_t, size_t>> orin_cost;
    // 构造代价矩阵（使用2DIOU作为距离度量）
    for (size_t i = 0; i < meas_list.size(); i++)
    {
        for (size_t j = 0; j < trace_list.size(); j++)
        {
            T iou_ = IOU_2D(trace_list.at(j), meas_list.at(i));

            (*optimizer_->costs())(i, j) = 1 - iou_; // NOTE
        }
    }

    optimizer_->costs()->Resize(meas_list.size(), trace_list.size());
    optimizer_->Minimize(&assignment);
    // optimizer_.PrintMatrix();

    return assignment;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
void SimpleTracker::trace_update(std::vector<rect_basic_struct> &detections,
                                 std::vector<size_t> &trace_assignment)
{
    SimpleTrack::PositionMeasurement_3DBBox<T> meas_;
    rect_basic_struct *det;

    // std::cout << "----- Do trace_update -----" << std::endl;

    size_t trace_idx = 0;
    for (auto &sub_trace : TraceList)
    {
        if (trace_assignment.at(trace_idx) != 255)
        {
            det = &detections.at(trace_assignment.at(trace_idx));

            meas_.x() = det->center_pos[0];
            meas_.y() = det->center_pos[1];
            meas_.z() = det->center_pos[2];
            meas_.len() = det->box_len;
            meas_.wid() = det->box_wid;
            meas_.height() = det->box_height;
            meas_.theta() = det->heading;

            // std::cout << "meas info: " << meas_.x() << ", " << meas_.y() << ", " << meas_.z() << ", " << std::endl;
            // std::cout << "old x info: " << sub_trace.filter.getState().x() << ", " << sub_trace.filter.getState().y() << ", " << sub_trace.filter.getState().z() << ", "
            //           << sub_trace.filter.getState().vx() << ", " << sub_trace.filter.getState().vy() << ", " << sub_trace.filter.getState().vz() << ", "
            //           << sub_trace.filter.getState().ax() << ", " << sub_trace.filter.getState().ay() << ", " << sub_trace.filter.getState().az() << ", " << std::endl;
            
            sub_trace.Update(meas_);

            // std::cout << "new x info: " << sub_trace.filter.getState().x() << ", " << sub_trace.filter.getState().y() << ", " << sub_trace.filter.getState().z() << ", "
            //           << sub_trace.filter.getState().vx() << ", " << sub_trace.filter.getState().vy() << ", " << sub_trace.filter.getState().vz() << ", "
            //           << sub_trace.filter.getState().ax() << ", " << sub_trace.filter.getState().ay() << ", " << sub_trace.filter.getState().az() << ", " << std::endl;

            sub_trace.Age++;
            sub_trace.unmatched_coun = 0;
            sub_trace.matched_count++;
        }
        else
        {
            sub_trace.matched_count = 0;
            sub_trace.unmatched_coun++;
        }

        trace_idx++;
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void SimpleTracker::build_new_trace(std::vector<rect_basic_struct> &detections,
                                    std::vector<size_t> &meas_assignment)
{
    // std::cout << "build_new_trace" << std::endl;

    for (size_t i = 0; i < meas_assignment.size(); ++i)
    {
        if (meas_assignment.at(i) == 1) //
        {
            SimpleTrack::State<T> new_s;
            new_s.x() = detections.at(i).center_pos[0];
            new_s.y() = detections.at(i).center_pos[1];
            new_s.z() = detections.at(i).center_pos[2];
            new_s.vx() = 0.0;
            new_s.vy() = 0.0;
            new_s.vz() = 0.0;
            new_s.ax() = 0.0;
            new_s.ay() = 0.0;
            new_s.az() = 0.0;
            new_s.len() = detections.at(i).box_len;
            new_s.wid() = detections.at(i).box_wid;
            new_s.height() = detections.at(i).box_height;
            new_s.theta() = detections.at(i).heading;

            SimpleTrack::Trace<T> new_trace(new_s, global_ID);
            global_ID++;

            // std::cout << " new Trace Position [x, y]: " << new_s.x() << ", " << new_s.y() << std::endl;

            TraceList.push_back(new_trace);
        }
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<SimpleTrack::Trace<T>>} &TraceList
 * @return {*}
 */
std::vector<rect_basic_struct> SimpleTracker::build_2dbox_from_trace(void)
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

        // std::cout << "[x, y]: " << new_trace_box.center_pos[0] << ", " << new_trace_box.center_pos[1] << std::endl;

        trace_box_list.push_back(new_trace_box);
    }

    return trace_box_list;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void SimpleTracker::trace_management(void)
{
    std::vector<SimpleTrack::Trace<T>> TraceList_tmp;
    for (auto &sub_trace : TraceList)
    {
        if (!sub_trace.valid)
        {
            if (sub_trace.matched_count >= 3)
            {
                sub_trace.valid = true;
                TraceList_tmp.push_back(sub_trace);
            }
            else
            {
                if (sub_trace.Age < 5)
                {
                    TraceList_tmp.push_back(sub_trace);
                }
            }
        }
        else
        {
            if (sub_trace.unmatched_coun < 3)
            {
                TraceList_tmp.push_back(sub_trace);
            }
            else
            {
                sub_trace.valid = false;
            }
        }
    }

    // std::cout << "old trace num: " << TraceList.size()
    //           << " new trace num: " << TraceList_tmp.size() << std::endl;

    std::swap(TraceList, TraceList_tmp);

    // for (const auto &trace : TraceList)
    // {
    //     std::cout << "ID: " << trace.ID << " Age: " << trace.Age << "[x, y]"
    //               << trace.filter.getState().x() << ", " << trace.filter.getState().y() << std::endl;
    // }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<rect_basic_struct>} &detctions
 * @return {*}
 */
void SimpleTracker::run(std::vector<rect_basic_struct> &detections)
{
    std::cout << "Tracker Run" << std::endl;

    std::vector<size_t> trace_assignment, meas_assignment;

    // Step 1: Global Trace Predict
    trace_predict();

    // Step 2: Trace and Measments Assignment
    trace_meas_assignment(detections, trace_assignment, meas_assignment);

    // Step 3: Global Trace Update
    trace_update(detections, trace_assignment);

    // Step 4: Build New Traces
    build_new_trace(detections, meas_assignment);

    // Step 5: Global Trace Management
    trace_management();
}
