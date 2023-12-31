
#include "modules/perception/lidar/lib/tracker/hm_tracker/hm_tracker.h"

#include <chrono>
#include <random>

#include "Optics.hpp"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/common/lidar_object.h"
#include "modules/perception/lidar/common/lidar_timer.h"

namespace apollo {
namespace perception {
namespace lidar {

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MultiTargetTrackerInitOptions} &options
 * @return {*}
 */
bool HmObjectTracker::Init(const MultiTargetTrackerInitOptions &options) {
  // tracker_.reset(new lbk::Track());
  transform_vel_.reset(new apollo::perception::onboard::transform_vel());

  MapManagerInitOptions map_manager_init_options;
  if (!map_manager_.Init(map_manager_init_options)) {
    AINFO << "Failed to init map manager.";
  }

  optimizer_ = new apollo::perception::common::HungarianOptimizer<float>();
  optimizer_->costs()->Reserve(100, 100);

  return true;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
bool HmObjectTracker::Track(const MultiTargetTrackerOptions &options,
                            LidarFrame *frame) {
  if (frame == nullptr) {
    AERROR << "Input null frame ptr.";
    return false;
  }

  static double last_time = 0.0;
  double diff_time = 0.0;

  diff_time = frame->timestamp - last_time;
  last_time = frame->timestamp;

  if(diff_time > 0.15){
    AWARN << "WARNING, diff_time is too big: [" << diff_time << "]";
  }

  if ((diff_time > 0.3) || (diff_time < 0.0)) {
    // 时间戳异常，清空所有航迹
    AWARN << "WARNING, diff_time is too big: [" << diff_time << "]";
    track_list.clear();
    return true;
  }

  if (FLAGS_lidar_tracking) {
    lidar_tracking_proc(*frame, diff_time);
  } else {
    // 直接输出检测结果
    output_detection_result(*frame);
  }

  return true;
}

/**
 * @names: output_detection_result
 * @description: 直接输出原始检测结果
 * @param {LidarFrame} &frame
 * @return {*}
 */
void HmObjectTracker::output_detection_result(LidarFrame &frame) {
  frame.tracked_objects.clear();
  for (auto det : frame.segmented_objects) {
    std::shared_ptr<LidarObject> obj(new LidarObject());

    obj->track_id = det->id;

    // 位置、大小和朝向
    obj->center =
        Eigen::Vector3d(det->center.x(), det->center.y(), det->center.z());
    obj->size = Eigen::Vector3f(det->size.x(), det->size.y(), det->size.z());
    obj->theta = det->theta;

    // 目标类型及置信度
    obj->type = static_cast<base::ObjectDetType>(det->type);
    obj->confidence = det->confidence;

    frame.tracked_objects.emplace_back(obj);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {LidarFrame} &frame
 * @return {*}
 */
void HmObjectTracker::lidar_tracking_proc(LidarFrame &frame, const double dt) {
  Timer timer;
  std::vector<box_t>().swap(trace_box_list);
  std::vector<box_t>().swap(det_box_list);

  // 航迹一步预测
  lidar_trace_predict(frame, dt);

  // 构造航迹box
  creat_trace_box(trace_box_list);

  // 构造量测box
  creat_det_box(frame, det_box_list);

  // 分配
  match_trace_wiht_det(frame, trace_box_list, det_box_list);

  // 航迹更新
  track_update_all();

  track_simple_classifity();

  // 航迹起始
  track_managment(det_box_list);

  compute_shape4out(frame);

  trans_obj2frame(frame);

  // 释放内存
  std::vector<box_t>().swap(trace_box_list);
  std::vector<box_t>().swap(det_box_list);
  track_list.shrink_to_fit();
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {LidarFrame} &frame
 * @return {*}
 */
void HmObjectTracker::trans_obj2frame(LidarFrame &frame) {
  frame.tracked_objects.clear();

  for (auto &trace : track_list) {
    if (trace.track_manage.age == 1) {
      trace.track_manage.birth_time = frame.timestamp;  //
    }

    trace.track_manage.latest_tracked_time = frame.timestamp;  // 刷新最新时间戳
    trace.track_manage.tracking_time =
        trace.track_manage.latest_tracked_time -
        trace.track_manage.birth_time;  // 更新航迹跟踪时长

    if (trace.track_manage.track_status == TRK_Confirmed) {
      std::shared_ptr<LidarObject> obj(new LidarObject());
      obj->track_id = trace.track_manage.id;
      obj->center = Eigen::Vector3d(trace.X_(iDisLong), trace.X_(iDisLat),
                                    trace.X_(iDisHeight));
      obj->center_uncertainty =
          trace.P_.block<3, 3>(iDisLong, iDisLong).cast<float>();

      obj->size << trace.track_manage.shape4out.len,
          trace.track_manage.shape4out.wid, trace.track_manage.shape4out.height;
      obj->theta = trace.track_manage.shape4out.theta;
      obj->direction =
          Eigen::Vector3f(trace.track_manage.shape4out.theta, 0, 0);

      obj->velocity = trace.X_.segment<3>(iVreLong).cast<float>();
      obj->velocity_uncertainty =
          trace.P_.block<3, 3>(iVreLong, iVreLong).cast<float>();
      obj->acceleration = trace.X_.segment<3>(iAccLong).cast<float>();
      obj->acceleration_uncertainty =
          trace.P_.block<3, 3>(iAccLong, iAccLong).cast<float>();

      obj->tracking_time = trace.track_manage.tracking_time;
      obj->latest_tracked_time = trace.track_manage.latest_tracked_time;

      obj->type =
          static_cast<base::ObjectDetType>(trace.track_manage.type_manage.type);
      obj->confidence = trace.track_manage.type_manage.score;

      obj->motion_state = base::MotionState::UNKNOWN;
      switch (trace.track_manage.motion_status) {
        case motion_enum::moving_motion:
        case motion_enum::oncoming_motion:
        case motion_enum::away_motion:
          obj->motion_state = base::MotionState::MOVING;
          break;
        case motion_enum::stop_motion:
          obj->motion_state = base::MotionState::STOPPED;
          break;
        case motion_enum::stationary_motion:
          obj->motion_state = base::MotionState::STATIONARY;
          break;
        default:
          break;
      }

      obj->theta_uncertainty = trace.shape_vari[3];
      obj->size_uncertainty << trace.shape_vari[0], trace.shape_vari[1],
          trace.shape_vari[2];

      frame.tracked_objects.emplace_back(obj);

      if (0) {
        AINFO << "trace id:[ " << trace.track_manage.id << " ], "
              << "position:[ " << trace.X_.segment<3>(0).transpose() << "], "
              << " shape:[ " << obj->theta << ", " << obj->size.transpose()
              << "]"
              << " vel:[" << trace.X_.segment<3>(7).transpose() << " acc:["
              << trace.X_.segment<3>(10).transpose() << "]"
              << "type:[ " << objInnerType(trace.track_manage.type_manage.type)
              << " ], "
              << "score:[ " << obj->confidence << " ], "
              << "motion_state:[ " << objMotionStatus(obj->motion_state) << "]"
              << std::endl;
      }
    }
  }

  if (0) {
    for (auto det : det_box_list) {
      std::shared_ptr<LidarObject> obj(new LidarObject());

      if (!det.dust_) {
        obj->track_id = det.id;
        obj->center =
            Eigen::Vector3d(det.rect.center_pos[0], det.rect.center_pos[1],
                            det.rect.center_pos[2]);
        obj->size = Eigen::Vector3f(det.rect.box_len, det.rect.box_wid,
                                    det.rect.box_height);
        obj->theta = det.rect.heading;
        obj->type = base::ObjectDetType::UNKNOWN;
        frame.tracked_objects.emplace_back(obj);
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MotionState} &motion_status
 * @return {*}
 */
std::string HmObjectTracker::objMotionStatus(
    const base::MotionState &motion_status) {
  std::string motion_str = "UNKNOWN";
  switch (motion_status) {
    case base::MotionState::UNKNOWN:
      motion_str = "UNKNOWN";
      break;

    case base::MotionState::MOVING:
      motion_str = "MOVING";
      break;

    case base::MotionState::STATIONARY:
      motion_str = "STATIONARY";
      break;

    case base::MotionState::STOPPED:
      motion_str = "STOPPED";
      break;

    default:
      break;
  }

  return motion_str;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {inner_type_enum} type
 * @return {*}
 */
std::string HmObjectTracker::objInnerType(const inner_type_enum type) {
  std::string type_str = "UNKNOWN";
  switch (type) {
    case inner_type_enum::VEHICLE:
      type_str = "VEHICLE";
      break;

    case inner_type_enum::PEDESTRIAN:
      type_str = "PEDESTRIAN";
      break;

    case inner_type_enum::BARRIAR:
      type_str = "BARRIAR";
      break;

    case inner_type_enum::TRAFFICLIGHT:
      type_str = "TRAFFICLIGHT";
      break;

    case inner_type_enum::TRAFFICCONE:
      type_str = "TRAFFICCONE";
      break;

    case inner_type_enum::BICYCLE:
      type_str = "BICYCLE";
      break;

    default:
      break;
  }

  return type_str;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::lidar_trace_predict(LidarFrame &frame, double dt) {
  bool vehicle_flag = vehicle_status_update(frame);

  // AINFO << "vehicle_flag: [ " << vehicle_flag << " ]" << std::endl;

  for (auto &sub_trace : track_list) {
    sub_trace.trace_predict(dt);

    sub_trace.matched_info.sum_IOU = 0.0;
    sub_trace.matched_info.matched_case = no_matched;
    sub_trace.matched_info.matched_det_list.clear();

    sub_trace.vehilce_info.vehicle_flag = vehicle_flag;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<box_t>} &det_box_list
 * @return {*}
 */
void HmObjectTracker::creat_det_box(LidarFrame &frame,
                                    std::vector<box_t> &det_box_list) {
  uint32_t id = 0;
  Eigen::Matrix4d rota_mat = frame.lidar2world_pose.matrix();

  for (auto obj : frame.segmented_objects) {  // 获取检测结果
    box_t sub_det_box;
    sub_det_box.id = id;

    sub_det_box.rect.center_pos[0] = obj->center.x();
    sub_det_box.rect.center_pos[1] = obj->center.y();
    sub_det_box.rect.center_pos[2] = obj->center.z();

    sub_det_box.rect.box_len = obj->size.x();
    sub_det_box.rect.box_wid = obj->size.y();
    sub_det_box.rect.box_height = obj->size.z();

    sub_det_box.rect.heading = obj->theta;

    sub_det_box.score = obj->confidence;
    sub_det_box.type = static_cast<inner_type_enum>(obj->type);

    // AINFO << "detection object Type: " << static_cast<int>(obj->type);

    if (USING_MODEL_DETECTION) {
      sub_det_box.valid = true;
      sub_det_box.dust_ = (!sub_det_box.valid);

      // 对于红绿灯、行人、障碍物等目标，强制拟合成theta为0的矩形
      if ((sub_det_box.type == inner_type_enum::PEDESTRIAN) ||
          (sub_det_box.type == inner_type_enum::BARRIAR) ||
          (sub_det_box.type == inner_type_enum::TRAFFICLIGHT) ||
          (sub_det_box.type == inner_type_enum::TRAFFICCONE)) {
        // 重新构造长宽以及theta
        re_compute_shape(sub_det_box);
      }

    } else {
      sub_det_box.valid = (recognise_dust(obj));
      sub_det_box.dust_ = (!sub_det_box.valid);

      // 重新构造长宽以及theta
      re_compute_shape(sub_det_box);
    }

    // 判定目标是否处于室内(注意函数结果取反)
    sub_det_box.valid = (!(detecObjIndoor(sub_det_box, rota_mat)));

    det_box_list.push_back(sub_det_box);
    id++;
  }
}

/**
 * @names: detecObjIndoor
 * @description: 判定目标是否处于室内
 * @param {box_t} &det_box
 * @param {Matrix4d} lidar2world
 * @return {*}
 */
bool HmObjectTracker::detecObjIndoor(const box_t &det_box,
                                     const Eigen::Matrix4d lidar2world) {
  // 目标在室内外判定逻辑
  Eigen::Vector4d objInWord;

  double offset_[4][2] = {{1.0, 1.0}, {-1.0, 1.0}, {-1.0, -1.0}, {1.0, -1.0}};
  double half_wid = det_box.rect.box_wid * 0.5;
  double half_len = det_box.rect.box_len * 0.5;

  double RotMat[2][2] = {
      {cos(det_box.rect.heading), sin(det_box.rect.heading)},
      {-1.0 * sin(det_box.rect.heading), cos(det_box.rect.heading)}};
  double tempX, tempY;
  for (int idx = 0; idx < 4; idx++) {
    tempX = det_box.rect.center_pos[0] +
            (offset_[idx][0] * half_wid) * RotMat[0][0] +
            (offset_[idx][1] * half_len) * RotMat[0][1];

    tempY = det_box.rect.center_pos[1] +
            (offset_[idx][0] * half_wid) * RotMat[1][0] +
            (offset_[idx][1] * half_len) * RotMat[1][1];

    objInWord << tempX, tempY, det_box.rect.center_pos[2], 1.0;

    objInWord = lidar2world * objInWord;

    // 有一个角点在室外，则目标在室外
    if (!(map_manager_.IsIndoor(objInWord(0), objInWord(1), objInWord(2)))) {
      return false;
    }
  }

  return true;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {box_t} &sub_det_box
 * @return {*}
 */
void HmObjectTracker::re_compute_shape(box_t &sub_det_box) {
  Eigen::MatrixXd orin_pts(4, 2);
  Eigen::Matrix2d rota_mat(2, 2);
  Eigen::MatrixXd rota_pts(2, 4);

  double half_len = sub_det_box.rect.box_len * 0.5,
         half_wid = sub_det_box.rect.box_wid * 0.5;

  orin_pts << +half_len, +half_wid, +half_len, -half_wid, -half_len, -half_wid,
      -half_len, +half_wid;

  rota_mat << cos(sub_det_box.rect.heading), -sin(sub_det_box.rect.heading),
      sin(sub_det_box.rect.heading), cos(sub_det_box.rect.heading);

  rota_pts = rota_mat * orin_pts.transpose();

  // 重新计算长宽以及theta
  auto max_ = rota_pts.rowwise().maxCoeff();
  auto min_ = rota_pts.rowwise().minCoeff();

  if ((max_(0) - min_(0)) > (max_(1) - min_(1))) {
    sub_det_box.rect.box_len = max_(0) - min_(0);
    sub_det_box.rect.box_wid = max_(1) - min_(1);
    sub_det_box.rect.heading = 0.0;
  } else {
    sub_det_box.rect.box_len = max_(1) - min_(1);
    sub_det_box.rect.box_wid = max_(0) - min_(0);
    sub_det_box.rect.heading = M_PI_2;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {shared_ptr<LidarObject>} &det
 * @return {*}
 */
bool HmObjectTracker::recognise_dust(std::shared_ptr<LidarObject> &det) {
  // STEP 1: 统计低反点个数：分成三挡 [<2, < 10, > 10]
  // int total_num = det->cloud.size();
  bool is_valid = true;
  double low_num, mid_num, hign_num;
  low_num = mid_num = hign_num = 0.0;
  for (auto point : det->cloud) {
    if (point.intensity < 2) {
      low_num += 1.0;
    } else if (point.intensity < 10) {
      mid_num += 1.0;
    } else {
      hign_num += 1.0;
    }
  }

  // 统计该box是否“浮选”，即Z的下边缘离地（0m）是多少
  double dis_to_ground = det->center.z() - 0.5 * det->size.z();
  double low_num_per = low_num / det->cloud.size();
  double range_ = sqrt(pow(det->center.x(), 2.0) + pow(det->center.y(), 2.0));

  if (((dis_to_ground > 0.5) && (low_num_per > 0.9)) ||
      ((range_ < 10.0) && (low_num_per > 0.95))) {
    is_valid = false;
  }

  return is_valid;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<box_t>} &trace_box_list
 * @return {*}
 */
void HmObjectTracker::creat_trace_box(std::vector<box_t> &trace_box_list) {
  for (auto sub_trace : track_list) {
    box_t sub_trace_box;
    sub_trace_box.rect.center_pos[0] = sub_trace.X_(iDisLong);
    sub_trace_box.rect.center_pos[1] = sub_trace.X_(iDisLat);
    sub_trace_box.rect.center_pos[2] = sub_trace.X_(iDisHeight);

    sub_trace_box.rect.box_len = sub_trace.X_(iBoxLen);
    sub_trace_box.rect.box_wid = sub_trace.X_(iBoxWid);

    sub_trace_box.rect.box_height = sub_trace.X_(iBoxHeight);
    sub_trace_box.rect.heading = sub_trace.X_(iBoxHeading);

    trace_box_list.push_back(sub_trace_box);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::match_trace_wiht_det(LidarFrame &frame,
                                           std::vector<box_t> &trace_box_list,
                                           std::vector<box_t> &det_box_list) {
  std::vector<greedy_match_info_t> cost_matrix;

  // 构造代价矩阵
  cost_matrix = build_cost_matrix(trace_box_list, det_box_list);

  if (USING_MODEL_DETECTION) {
    assignment_used_for_model(frame, cost_matrix, trace_box_list, det_box_list);
  } else {
    assignment(frame, cost_matrix, trace_box_list, det_box_list);
  }
}

/**
 * @names: 构造代价矩阵: IOU + 中心欧氏距离
 * @description: Briefly describe the function of your function
 * @return {*}
 */
std::vector<greedy_match_info_t> HmObjectTracker::build_cost_matrix(
    std::vector<box_t> &trace_box_list, std::vector<box_t> &det_box_list) {
  std::vector<greedy_match_info_t> cost_map;

  if ((trace_box_list.size() == 0) || (det_box_list.size() == 0)) {
    return cost_map;
  }

  // 构造代价矩阵
  uint32_t trace_idx = 0;
  for (auto sub_trace : trace_box_list) {
    for (auto &sub_det : det_box_list) {
      greedy_match_info_t sub_cost_info;

      sub_cost_info.trace_idx = trace_idx;
      sub_cost_info.det_idx = sub_det.id;

      if (sub_det.valid == true) {
        sub_cost_info.iou = IOU_2D(sub_trace.rect, sub_det.rect);
        if ((track_list.at(trace_idx).track_manage.track_status ==
             TRK_Confirmed) &&
            (sub_cost_info.iou > 0.0)) {
          sub_cost_info.iou += 1.0;
          sub_det.iou_count++;
        }
      } else {
        sub_cost_info.iou = 0.0;
      }

      sub_cost_info.center_dis =
          sqrt(pow((sub_trace.rect.center_pos[0] - sub_det.rect.center_pos[0]),
                   2.0) +
               pow((sub_trace.rect.center_pos[1] - sub_det.rect.center_pos[1]),
                   2.0));

      cost_map.push_back(sub_cost_info);
    }
    trace_idx++;
  }

  return cost_map;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::assignment(
    LidarFrame &frame, const std::vector<greedy_match_info_t> &cost_matrix,
    std::vector<box_t> &trace_box_list, std::vector<box_t> &det_box_list) {
  if ((trace_box_list.size() == 0) || (det_box_list.size() == 0)) {
    return;
  }

  // 执行greedy（先阶段使用greedy，后续可修改为Hungarian)
  std::vector<std::vector<uint32_t>> trace_matched_info(trace_box_list.size());

  // 第一次分配
  uint32_t det_idx_, trace_idx_;
  for (const auto &cost_info : cost_matrix) {
    det_idx_ = cost_info.det_idx;
    trace_idx_ = cost_info.trace_idx;

    if (det_box_list.at(det_idx_).valid || det_box_list.at(det_idx_).reues_) {
      if (cost_info.iou > 0.0) {
        if ((!det_box_list.at(det_idx_).valid) &&
            (track_list.at(trace_idx_).track_manage.track_status ==
             TRK_Detected)) {
          continue;
        }
        double trace_size = trace_box_list.at(trace_idx_).rect.box_len *
                            trace_box_list.at(trace_idx_).rect.box_wid;

        double box_size = det_box_list.at(det_idx_).rect.box_len *
                          det_box_list.at(det_idx_).rect.box_wid;
        double real_iou =
            (cost_info.iou > 1.0) ? (cost_info.iou - 1.0) : cost_info.iou;

        double overlap_ = (trace_size + box_size) * real_iou / (1.0 + real_iou);

        double overlap_of_small_one = (box_size <= trace_size)
                                          ? (overlap_ / box_size)
                                          : (overlap_ / trace_size);

        // 判定高度上是否有交集
        double diff_height = (trace_box_list.at(trace_idx_).rect.center_pos[2] -
                              det_box_list.at(det_idx_).rect.center_pos[2]);

        if (fabs(diff_height) <
            (0.5 * (trace_box_list.at(trace_idx_).rect.box_height +
                    det_box_list.at(det_idx_).rect.box_height))) {
          bool matched = false;

          // 该量测唯一与该航迹相交，且对象航迹为确认航迹，则直接关联
          if ((track_list.at(trace_idx_).track_manage.track_status ==
               TRK_Confirmed) &&
              (det_box_list.at(det_idx_).iou_count == 1) && (real_iou > 0.0)) {
            matched = true;

            // 关联到唯一量测flag
            if (trace_matched_info.at(trace_idx_).empty()) {
              track_list.at(trace_idx_).matched_info.has_high_meas = true;
            }
          } else {
            if ((real_iou > 0.3) || (overlap_of_small_one > 0.5)) {
              if (trace_matched_info.at(trace_idx_).empty()) {
                matched = true;
              } else {
                double overlap_ =
                    (trace_size + box_size) * real_iou / (1.0 + real_iou);

                if ((box_size < trace_size) && ((overlap_ / box_size) > 0.8)) {
                  matched = true;
                }
              }
            }
          }

          if (matched) {
            trace_matched_info.at(trace_idx_).push_back(det_idx_);
            track_list.at(trace_idx_).matched_info.matched_case = normal_case;
            track_list.at(trace_idx_).matched_info.sum_IOU += real_iou;

            det_box_list.at(det_idx_).valid = false;
            det_box_list.at(det_idx_).reues_ = false;
          } else {
            // 20230804: 检测目标匹配情况
            if (DEBUG_SWITCH) {
              AINFO << "Asso Failed: [ "
                    << track_list.at(trace_idx_).track_manage.id << ", "
                    << det_idx_ << "]"
                    << " real_iou : " << real_iou
                    << " iou_count: " << det_box_list.at(det_idx_).iou_count;
            }
          }
        }
      }
    }
  }

  // 整理航迹关联检测情况
  for (uint32_t trace_idx = 0; trace_idx < track_list.size(); trace_idx++) {
    auto match_info = trace_matched_info.at(trace_idx);
    auto &sub_trace = track_list.at(trace_idx);

    if (match_info.size() == 1) {
      sub_trace.matched_info.matched_det_list.push_back(
          det_box_list.at(match_info.at(0)));
    } else {
      // TODO: refitting
      if (match_info.size() > 1) {
        if (sub_trace.matched_info.has_high_meas) {
          sub_trace.matched_info.has_high_meas = false;
        }

        box_t new_det_box;
        refitting_det_box(frame, sub_trace.X_, match_info, det_box_list,
                          new_det_box);

        sub_trace.matched_info.matched_det_list.push_back(new_det_box);
      }
    }
  }
}

/**
 * @names: assignment_used_for_model
 * @description: 适配模型作为检测前端的分配算法
 * @param {LidarFrame} &frame
 * @param {vector<greedy_match_info_t>} &cost_matrix
 * @param {  } std
 * @param {vector<box_t>} &det_box_list
 * @return {*}
 */
void HmObjectTracker::assignment_used_for_model(
    LidarFrame &frame, std::vector<greedy_match_info_t> &cost_matrix,
    std::vector<box_t> &trace_box_list, std::vector<box_t> &det_box_list) {
  if ((trace_box_list.size() == 0) || (det_box_list.size() == 0)) {
    return;
  }

  const bool USING_HM = true;

  if (USING_HM) {
    // Step 1: 使用HM算法(全局)
    for (uint i1 = 0; i1 < det_box_list.size(); ++i1) {
      for (uint i2 = 0; i2 < trace_box_list.size(); ++i2) {
        float iou_ = cost_matrix.at(i2 * det_box_list.size() + i1).iou;
        // AINFO << "[" << i1 << "," << i2 << "]: " << iou_;
        if (iou_ > 1.0) {
          (*optimizer_->costs())(i1, i2) = 1.0 - (iou_ - 1.0);
        } else {
          (*optimizer_->costs())(i1, i2) = 1.0 - (iou_);
        }
      }
    }
    optimizer_->costs()->Resize(det_box_list.size(), trace_box_list.size());

    // 执行分配
    std::vector<std::pair<size_t, size_t>> assignments;
    optimizer_->Minimize(&assignments);

    uint32_t det_idx, trace_idx;
    for (const auto &assignment : assignments) {
      det_idx = assignment.first;
      trace_idx = assignment.second;
      if (cost_matrix.at(trace_idx * det_box_list.size() + det_idx).iou > 0.2) {
        track_list.at(trace_idx).matched_info.matched_det_list.push_back(
            det_box_list.at(det_idx));
        track_list.at(trace_idx).matched_info.matched_case = normal_case;

        det_box_list.at(det_idx).valid = false;
      }
    }

    // Step 2: 对于未关联到目标的行人航迹以及未被分配的行人检测，进行二次分配
    std::sort(cost_matrix.begin(), cost_matrix.end(), cost_compare);

    for (const auto &cost_info : cost_matrix) {
      det_idx = cost_info.det_idx;
      trace_idx = cost_info.trace_idx;

      if (det_box_list.at(det_idx).valid &&
          track_list.at(trace_idx).matched_info.matched_det_list.empty()) {
        if ((det_box_list.at(det_idx).type == inner_type_enum::PEDESTRIAN) &&
            (track_list.at(trace_idx).track_manage.type_manage.type ==
             inner_type_enum::PEDESTRIAN)) {
          if (cost_info.center_dis < 2.0) {
            det_box_list.at(det_idx).valid = false;
            track_list.at(trace_idx).matched_info.matched_det_list.push_back(
                det_box_list.at(det_idx));
            track_list.at(trace_idx).matched_info.matched_case = normal_case;
          }
        }
      }
    }
  } else {
    // 执行greedy
    // 先排序后分配
    std::sort(cost_matrix.begin(), cost_matrix.end(), cost_compare);
    std::vector<std::vector<uint32_t>> trace_matched_info(
        trace_box_list.size());

    // 第一次分配
    uint32_t det_idx_, trace_idx_;
    for (const auto &cost_info : cost_matrix) {
      det_idx_ = cost_info.det_idx;
      trace_idx_ = cost_info.trace_idx;

      if (det_box_list.at(det_idx_).valid &&
          trace_matched_info.at(trace_idx_).empty()) {
        if (cost_info.iou > 0.0) {
          double trace_size = trace_box_list.at(trace_idx_).rect.box_len *
                              trace_box_list.at(trace_idx_).rect.box_wid;

          double box_size = det_box_list.at(det_idx_).rect.box_len *
                            det_box_list.at(det_idx_).rect.box_wid;
          double real_iou =
              (cost_info.iou > 1.0) ? (cost_info.iou - 1.0) : cost_info.iou;

          double overlap_ =
              (trace_size + box_size) * real_iou / (1.0 + real_iou);

          double overlap_of_small_one = (box_size <= trace_size)
                                            ? (overlap_ / box_size)
                                            : (overlap_ / trace_size);

          bool matched =
              ((real_iou > 0.3) && (overlap_of_small_one > 0.5)) ? true : false;

          if (matched) {
            trace_matched_info.at(trace_idx_).push_back(det_idx_);
            track_list.at(trace_idx_).matched_info.matched_case = normal_case;
            track_list.at(trace_idx_).matched_info.sum_IOU += real_iou;

            det_box_list.at(det_idx_).valid = false;
            det_box_list.at(det_idx_).reues_ = false;
          }
        }
      }
    }

    // 第二次分配
    for (const auto &cost_info : cost_matrix) {
      det_idx_ = cost_info.det_idx;
      trace_idx_ = cost_info.trace_idx;

      if ((det_box_list.at(det_idx_).valid) &&
          (trace_matched_info.at(trace_idx_).empty()) &&
          (track_list.at(trace_idx_).matched_info.matched_case == no_matched)) {
        // 中心点距离判定
        if (cost_info.center_dis < 2.0) {
          double trace_size = trace_box_list.at(trace_idx_).rect.box_len *
                              trace_box_list.at(trace_idx_).rect.box_wid;
          double box_size = det_box_list.at(det_idx_).rect.box_len *
                            det_box_list.at(det_idx_).rect.box_wid;
          double size_scale = (trace_size < box_size) ? (trace_size / box_size)
                                                      : (box_size / trace_size);

          if (size_scale > 0.2) {
            trace_matched_info.at(trace_idx_).push_back(det_idx_);
            det_box_list.at(det_idx_).valid = false;
            track_list.at(trace_idx_).matched_info.matched_case = normal_case;

            double real_iou =
                (cost_info.iou > 1.0) ? (cost_info.iou - 1.0) : cost_info.iou;
            track_list.at(trace_idx_).matched_info.sum_IOU += real_iou;

          } else {
            // 中心点距离超过5米后，后续trace-dex对不再处理
            break;
          }
        }
      }
    }

    // 整理航迹关联检测情况
    for (uint32_t trace_idx = 0; trace_idx < track_list.size(); trace_idx++) {
      auto match_info = trace_matched_info.at(trace_idx);
      auto &sub_trace = track_list.at(trace_idx);
      if (match_info.size() == 1) {
        sub_trace.matched_info.matched_det_list.push_back(
            det_box_list.at(match_info.at(0)));
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {LidarFrame} &frame
 * @param {VectorXd} trace_status
 * @param {  } std
 * @param {  } std
 * @param {box_t} &new_det_box
 * @return {*}
 */
void HmObjectTracker::refitting_det_box(LidarFrame &frame,
                                        const Eigen::VectorXd trace_status,
                                        std::vector<uint32_t> det_list,
                                        std::vector<box_t> &det_box_list,
                                        box_t &new_det_box) {
  Timer timer;
  uint32_t point_num = 0;
  for (auto idx : det_list) {
    point_num += frame.segmented_objects.at(idx)->cloud.size();

    auto obj = frame.segmented_objects.at(idx);
  }

  Eigen::MatrixXd pointSet(2, point_num);

  uint32_t p_idx = 0;
  int cur_idx = 0;
  double box_max_theta = 0;
  double box_max_size = 0.0;
  double z_center = 0.0, z_height = 0.0;
  for (auto idx : det_list) {
    cur_idx++;
    auto obj = frame.segmented_objects.at(idx);
    for (auto point : obj->cloud) {
      pointSet(0, p_idx) = point.y;
      pointSet(1, p_idx) = point.x;
      p_idx++;
    }

    // 使用面积最大的box的z属性作为最终的z属性
    if ((obj->size.x() * obj->size.y()) > box_max_size) {
      z_center = obj->center.z();
      z_height = obj->size.z();
      box_max_size = (obj->size.x() * obj->size.y());
      box_max_theta = obj->theta;
    }
  }

  // 使用L_Shape重新进行box拟合
  Rect_t fit_rect = L_shape_Fit_Proc(pointSet, box_max_theta);

  new_det_box.rect.center_pos[0] = fit_rect.corner[4][1];
  new_det_box.rect.center_pos[1] = fit_rect.corner[4][0];
  new_det_box.rect.center_pos[2] = z_center;

  new_det_box.rect.box_len = fit_rect.length;
  new_det_box.rect.box_wid = fit_rect.width;
  new_det_box.rect.box_height = z_height;

  new_det_box.rect.heading = fit_rect.theta;

  if (DEBUG_SWITCH) {
    AINFO << "L_shape_Fit_Proc new box info: \n"
          << "center:[ " << new_det_box.rect.center_pos[0] << ", "
          << new_det_box.rect.center_pos[1] << ", "
          << new_det_box.rect.center_pos[2] << "] \n shape:[ "
          << new_det_box.rect.box_len << ", " << new_det_box.rect.box_wid
          << "] \n theta:" << new_det_box.rect.heading / M_PI * 180.0;

    AINFO << "  _______________ End L_shape_Fit_Proc _______________ ";
  }
}

/**
 * @name:
 * @description:
 * @param {trace_meas_pair} &c1
 * @param {trace_meas_pair} &c2
 * @return {*}
 */
bool HmObjectTracker::cost_compare(const greedy_match_info_t &c1,
                                   const greedy_match_info_t &c2) {
  if (c1.iou > c2.iou) {
    return true;
  } else {
    if (c1.iou < c2.iou) {
      return false;
    } else {
      if (c1.center_dis < c2.center_dis) {
        return true;
      } else {
        return false;
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::track_update_all() {
  for (auto &trace : track_list) {
    trace.Update_();
  }
}

/**
 * @names: track_managment
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::track_managment(std::vector<box_t> &det_box_list) {
  static uint32_t id = 0;
  std::vector<simple_tracker> new_track_list;
  for (auto &trace : track_list) {
    trace.track_manage.age++;

    if (trace.track_manage.track_status == TRK_Detected) {
      if (trace.matched_info.matched_det_list.size() > 0) {
        trace.track_manage.continue_assigned_count++;
        trace.track_manage.unassigned_count = 0;

        if (trace.track_manage.continue_assigned_count >= 4) {
          trace.track_manage.track_status = TRK_Confirmed;
          trace.track_manage.unassigned_count = 0;
        }
      } else {
        trace.track_manage.continue_assigned_count = 0;
        trace.track_manage.unassigned_count++;
      }
    } else {
      if (trace.matched_info.matched_det_list.empty()) {
        trace.track_manage.unassigned_count += 1;
      } else {
        trace.track_manage.unassigned_count = 0;
      }
    }

    // 临时航迹连续两次未关联，则删除
    if ((trace.track_manage.track_status == TRK_Detected) &&
        (trace.track_manage.unassigned_count >= 2)) {
      continue;
    }

    // 确认航迹连续三次未关联，则删除
    if (trace.track_manage.unassigned_count >= 6) {
      // AINFO << "delete trace [ " << trace.track_manage.id << "]";

      // // 适当延长行人航迹消亡周期:作用对象为稳定跟踪的行人
      // if ((trace.track_manage.age > 20) &&
      //     (trace.track_manage.type_manage.type ==
      //      inner_type_enum::PEDESTRIAN)) {
      //   if (trace.track_manage.unassigned_count > 10) {
      //     continue;
      //   }
      // } else {
      //   continue;
      // }

      continue;
    }

    new_track_list.push_back(trace);
  }

  track_list = new_track_list;

  // 新生航迹管理
  for (auto det : det_box_list) {
    if (det.valid) {
      simple_tracker new_trace(det, id);
      track_list.push_back(new_trace);

      id++;
    }
  }
}

/**
 * @names: compute_shape4out
 * @description: 结合绝对速度信息修正目标车头朝向
 * @param {LidarFrame} &frame
 * @return {*}
 */
void HmObjectTracker::compute_shape4out(LidarFrame &frame) {
  if (frame.pose_ptr == nullptr) {
    AINFO << "Error frame.loct_ptr";

    for (auto &sub_trace : track_list) {
      sub_trace.track_manage.shape4out.len = sub_trace.X_(iBoxLen);
      sub_trace.track_manage.shape4out.wid = sub_trace.X_(iBoxWid);
      sub_trace.track_manage.shape4out.height = sub_trace.X_(iBoxHeight);
      sub_trace.track_manage.shape4out.theta = sub_trace.X_(iBoxHeading);

      sub_trace.compute_extended_info_vari();
    }

    return;
  }

  const double yawRate = frame.pose_ptr.get()->angular_velocity_vrf().z();
  const double vehicle_v = frame.pose_ptr.get()->linear_velocity_vrf().y();

  for (auto &sub_trace : track_list) {
    // 计算绝对速度
    trace_ComputeAbsVel(sub_trace, yawRate, vehicle_v);

    // 计算速度夹角以及运动状态
    // if (fabs(yawRate) < 0.05) {
      trace_UpdateMotionStatus(sub_trace);
    // }

    // 更新形状以及朝向角
    trace_UpdateHeading(sub_trace);

    // 计算各项方差
    sub_trace.compute_extended_info_vari();
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {LidarFrame} &frame
 * @return {*}
 */
bool HmObjectTracker::vehicle_status_update(LidarFrame &frame) {
  const double yawRate =
      frame.pose_ptr.get()->angular_velocity_vrf().z();  // 当前时刻yawRate
  const double vehicle_v =
      frame.pose_ptr.get()->linear_velocity_vrf().y();  // 当前时刻车速

  static double last_yawrate = 0.0, last_vehicle_v = 0.0;

  static uint turn_change_count = 0, turn_count = 0, speed_up = 0, speed_down = 0;

  if (fabs(yawRate - last_yawrate) > (M_PI_2 / 90.0 * 0.5)) {
    turn_change_count++;
  } else {
    turn_change_count = 0;
  }

  if(fabs(yawRate) > (M_PI_2 / 90.0 * 0.5))
  {
    turn_count++;
  }
  else{
    turn_count = 0;
  }

  last_yawrate = yawRate;

  if (fabs(vehicle_v - last_vehicle_v) > 0.05) {
    if (vehicle_v > last_vehicle_v) {
      speed_up++;
    } else {
      speed_down++;
    }
  } else {
    speed_up = 0;
    speed_down = 0;
  }
  last_vehicle_v = vehicle_v;

  if ((turn_count >= 2) || (turn_change_count >= 2) || (speed_up >= 5) || (speed_down >= 5)) {
    return true;
  } else {
    return false;
  }
}

/**
 * @names: trace_ComputeAbsVel
 * @description: 计算目标绝对速度
 * @param {simple_tracker} &trace
 * @param {double} yawRate
 * @param {double} speed
 * @return {*}
 */
void HmObjectTracker::trace_ComputeAbsVel(simple_tracker &trace,
                                          const double yawRate,
                                          const double speed) {
  Eigen::Matrix3d rotation_sensor;
  rotation_sensor << 0, -yawRate, 0, yawRate, 0, 0, 0, 0, 0;

  // 目标“本地”位置信息：[x,y,z]
  Eigen::Vector3d local_loc(trace.X_(iDisLong), trace.X_(iDisLat),
                            trace.X_(iDisHeight));

  // 目标“本地”速度信息：[vx,vy,vz]
  Eigen::Vector3d local_vel(trace.X_(iVreLong), trace.X_(iVreLat),
                            trace.X_(iVreHeight));

  // 计算由于旋转带来的速度
  Eigen::Vector3d angular_trans_speed = rotation_sensor * local_loc;

  Eigen::Vector3d vel2car = (local_vel + angular_trans_speed);

  // 目标绝对速度
  const Eigen::Vector3d car_linear_speed_ = Eigen::Vector3d(speed, 0.0, 0.0);
  Eigen::Vector3d abs_vel = vel2car + car_linear_speed_;

  if (trace.track_manage.age == 2) {
    trace.abs_Vel << abs_vel;
  } else {
    trace.abs_Vel += ((abs_vel - trace.abs_Vel) * 0.8);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {simple_tracker} &trace
 * @return {*}
 */
void HmObjectTracker::trace_UpdateMotionStatus(simple_tracker &trace) {
  Eigen::Vector3d abs_vel = trace.abs_Vel;
  double theta_v1 = 0.0;
  double abs_speed_val = sqrt(pow(abs_vel(0), 2.0) + pow(abs_vel(1), 2.0));
  motion_enum cur_motion = Unknow_motion;
  if (abs_speed_val > 1.0) {
    cur_motion = moving_motion;

    if (fabs(abs_vel(1)) < 0.5) {
      abs_vel(1) = 0.0;
    }

    if (fabs(abs_vel(0)) < 0.5) {
      abs_vel(0) = 0.0;
    }

    theta_v1 = atan2(abs_vel(1), abs_vel(0));

    if ((fabs(theta_v1) < (M_PI / 3.0)) ||
        (fabs(theta_v1) > (M_PI - M_PI / 3.0))) {
      if (abs_vel(0) < 0.0) {
        cur_motion = oncoming_motion;
      } else {
        cur_motion = away_motion;
      }
    }

  } else {
    cur_motion = stationary_motion;
  }

  // 航迹起始时刻
  if (trace.track_manage.motion_status == Unknow_motion) {
    if (trace.track_manage.track_status == TRK_Confirmed) {
      if (cur_motion == trace.track_manage.motion_status_last) {
        trace.track_manage.same_motion++;
        if (trace.track_manage.same_motion >= 5) {
          trace.track_manage.motion_status = cur_motion;
        }
      } else {
        trace.track_manage.same_motion = 0;
      }
      trace.track_manage.motion_status_last = cur_motion;
    }
  } else {
    // 当前状态是静止or停止状态
    if ((trace.track_manage.motion_status == stationary_motion) ||
        (trace.track_manage.motion_status == stop_motion)) {
      if (cur_motion != stationary_motion) {
        trace.track_manage.motion_diff += ((uint)(abs_speed_val / 2.0) + 1);
        if (trace.track_manage.motion_diff > 10) {
          trace.track_manage.motion_status = cur_motion;
          trace.track_manage.motion_diff = 0;
        }
      } else {
        trace.track_manage.motion_diff = 0;
      }
    } else {
      // 当前状态是运动
      // 运动状态往静止状态切换
      if (cur_motion == stationary_motion) {
        trace.track_manage.motion_diff++;
        if (trace.track_manage.motion_diff > 10) {
          trace.track_manage.motion_status = stop_motion;
          trace.track_manage.motion_diff = 0;
        }
      } else {
        // 同样是运动状态，但是运动方向不一致
        if (cur_motion != trace.track_manage.motion_status) {
          trace.track_manage.motion_diff += ((uint)(abs_speed_val / 2.0) + 1);

          if (((trace.track_manage.motion_status == oncoming_motion) &&
               (cur_motion == away_motion)) ||
              ((trace.track_manage.motion_status == away_motion) &&
               (cur_motion == oncoming_motion))) {
            if (trace.track_manage.motion_diff > 40) {
              trace.track_manage.motion_status = cur_motion;
              trace.track_manage.motion_diff = 0;
            }
          } else {
            if (trace.track_manage.motion_diff > 5) {
              trace.track_manage.motion_status = cur_motion;
              trace.track_manage.motion_diff = 0;
            }
          }
        } else {
          trace.track_manage.motion_diff = 0;
        }
      }
    }
  }
}

/**
 * @names: trace_UpdateHeading
 * @description: Briefly describe the function of your function
 * @param {simple_tracker} &sub_trace
 * @return {*}
 */
void HmObjectTracker::trace_UpdateHeading(simple_tracker &sub_trace) {
  Eigen::Vector3d abs_vel = sub_trace.abs_Vel;
  double theta_v = 0.0;
  double abs_speed_val = sqrt(pow(abs_vel(0), 2.0) + pow(abs_vel(1), 2.0));
  if (abs_speed_val > 0.8) {
    if (fabs(abs_vel(1)) < 0.5) {
      abs_vel(1) = 0.0;
    }

    if (fabs(abs_vel(0)) < 0.5) {
      abs_vel(0) = 0.0;
    }

    theta_v = atan2(abs_vel(1), abs_vel(0));
  }

  // step01: 按照长宽进行角度修正
  // 长一定要比宽大，否则进行角度旋转以及长宽翻转
  double tempTheta_phy, tempLen, tempWid, tempTheta2;
  if (sub_trace.X_(iBoxLen) < sub_trace.X_(iBoxWid)) {
    // 若长宽相差不大的情况下，即接近正方形时，不要进行长宽翻转
    if (fabs(sub_trace.X_(iBoxWid) - sub_trace.X_(iBoxLen)) >= 0.5) {
      tempLen = sub_trace.X_(iBoxWid);
      tempWid = sub_trace.X_(iBoxLen);

      tempTheta_phy = (sub_trace.X_(iBoxHeading) > 0.0)
                          ? (sub_trace.X_(iBoxHeading) - M_PI_2)
                          : (sub_trace.X_(iBoxHeading) + M_PI_2);
    } else {
      tempLen = sub_trace.X_(iBoxLen);
      tempWid = sub_trace.X_(iBoxWid);

      tempTheta_phy = sub_trace.X_(iBoxHeading);
    }
  } else {
    tempLen = sub_trace.X_(iBoxLen);
    tempWid = sub_trace.X_(iBoxWid);

    tempTheta_phy = sub_trace.X_(iBoxHeading);
  }

  // 小目标强制为180度
  if ((fabs(tempLen - tempWid) < 0.2) && (tempLen < 1.5) && (tempWid < 1.5) &&
      (sub_trace.track_manage.motion_status < moving_motion)) {
    // tempWid = tempLen;

    if (tempLen < tempWid) {
      tempLen = tempWid;
    } else {
      tempWid = tempLen;
    }

    tempTheta_phy = 0.0;
    // TODO 强制角度后，长宽属性没有正确修正
  }

  if (DEBUG_SWITCH) {
    AINFO << "new shape: [ " << tempLen << ", " << tempWid << " ]";
  }

  // double diff_theta;
  // double new_theta = 0.0;
  double diff_4[4] = {0.0, 0.0, 0.0, 0.0};
  std::vector<std::pair<float, float>> diff_vec_;
  double compared_theta = 0.0;

  // 步骤一：首先判定是否为静态目标-->只用形状theta
  if ((sub_trace.track_manage.motion_status == stationary_motion) ||
      (sub_trace.track_manage.motion_status == Unknow_motion)) {
    compared_theta = sub_trace.track_manage.shape4out.theta;
    if (DEBUG_SWITCH) {
      AINFO << "step1::::"
            << " trace ID:[" << sub_trace.track_manage.id << "] "
            << "compared_theta:[" << compared_theta / M_PI * 180.0 << "]";
    }
  } else {
    // 运动目标，寻找和运动方向差值最小的方向
    if (sub_trace.track_manage.motion_diff == 0) {
      if (sub_trace.track_manage.motion_status == stop_motion) {
        compared_theta = sub_trace.track_manage.shape4out.theta;
        if (DEBUG_SWITCH) {
          AINFO << "step2::::"
                << " trace ID:[" << sub_trace.track_manage.id << "] "
                << "compared_theta:[" << compared_theta / M_PI * 180.0 <<
                "]";
        }

      } else {
        compared_theta = theta_v;
        if (DEBUG_SWITCH) {
          AINFO << "step3::::"
                << " trace ID:[" << sub_trace.track_manage.id << "] "
                << "compared_theta:[" << compared_theta / M_PI * 180.0 <<
                "]";
        }
      }
    } else {
      compared_theta = sub_trace.track_manage.shape4out.theta;
      if (DEBUG_SWITCH) {
        AINFO << "step4::::"
              << " trace ID:[" << sub_trace.track_manage.id << "] "
              << "compared_theta:[" << compared_theta / M_PI * 180.0 << "]";
      }
    }
  }

  diff_4[0] = (compared_theta - tempTheta_phy);

  tempTheta2 =
      (tempTheta_phy > 0.0) ? (tempTheta_phy - M_PI) : (tempTheta_phy + M_PI);

  diff_4[1] = (compared_theta - tempTheta2);

  if (fabs(diff_4[0]) > M_PI) {
    diff_4[0] =
        (diff_4[0] > 0.0) ? (diff_4[0] - 2.0 * M_PI) : (diff_4[0] + 2.0 * M_PI);
  }
  if (fabs(diff_4[1]) > M_PI) {
    diff_4[1] =
        (diff_4[1] > 0.0) ? (diff_4[1] - 2.0 * M_PI) : (diff_4[1] + 2.0 * M_PI);
  }
  diff_4[0] = fabs(diff_4[0]);
  diff_4[1] = fabs(diff_4[1]);

  diff_vec_.push_back(std::make_pair(diff_4[0], tempTheta_phy));
  diff_vec_.push_back(std::make_pair(diff_4[1], tempTheta2));

  double tempTheta_phy_change = 0.0, tempTheta_phy_change_180 = 0.0;
  if (fabs(tempLen - tempWid) < 0.5) {
    tempTheta_phy_change = (tempTheta_phy > 0.0) ? (tempTheta_phy - M_PI_2)
                                                 : (tempTheta_phy + M_PI_2);
    tempTheta_phy_change_180 = (tempTheta_phy_change > 0.0)
                                   ? (tempTheta_phy_change - M_PI)
                                   : (tempTheta_phy_change + M_PI);

    diff_4[2] = (sub_trace.track_manage.shape4out.theta - tempTheta_phy_change);
    diff_4[3] =
        (sub_trace.track_manage.shape4out.theta - tempTheta_phy_change_180);

    if (fabs(diff_4[2]) > M_PI) {
      diff_4[2] = (diff_4[2] > 0.0) ? (diff_4[2] - 2.0 * M_PI)
                                    : (diff_4[2] + 2.0 * M_PI);
    }
    if (fabs(diff_4[3]) > M_PI) {
      diff_4[3] = (diff_4[3] > 0.0) ? (diff_4[3] - 2.0 * M_PI)
                                    : (diff_4[3] + 2.0 * M_PI);
    }

    diff_4[2] = fabs(diff_4[2]);
    diff_4[3] = fabs(diff_4[3]);

    diff_vec_.push_back(std::make_pair(diff_4[2], tempTheta_phy_change));
    diff_vec_.push_back(std::make_pair(diff_4[3], tempTheta_phy_change_180));
  }

  uint8_t min_idx = 0;
  float min_data = 1e3;
  for (uint8_t ii = 0; ii < diff_vec_.size(); ii++) {
    const auto &sub_ = diff_vec_.at(ii);
    if (sub_.first < min_data) {
      min_data = sub_.first;
      min_idx = ii;
    }
  }

  if (min_idx >= 2) {
    double temppp = tempLen;
    tempLen = tempWid;
    tempWid = temppp;
  }
  if (DEBUG_SWITCH) {
    AINFO << "theta_v: [" << theta_v / M_PI * 180.0 << "]"
          << "tempTheta_phy: [" << tempTheta_phy / M_PI * 180.0 << "]"
          << "tempTheta2: [" << tempTheta2 / M_PI * 180.0 << "]"
          << "tempTheta_phy_change: [" << tempTheta_phy_change / M_PI * 180.0
          << "]"
          << "tempTheta_phy_change_180: ["
          << tempTheta_phy_change_180 / M_PI * 180.0 << "]"
          << "final theta: [" << diff_vec_.at(min_idx).second / M_PI * 180.0
          << "]";
  }

  // 更新朝向角并约束其范围
  sub_trace.track_manage.shape4out.theta = diff_vec_.at(min_idx).second;
  if (fabs(sub_trace.track_manage.shape4out.theta) > 3.14159) {
    sub_trace.track_manage.shape4out.theta =
        (sub_trace.track_manage.shape4out.theta > 0.0)
            ? (sub_trace.track_manage.shape4out.theta - 2.0 * 3.14159)
            : (sub_trace.track_manage.shape4out.theta + 2.0 * 3.14159);
  }

  sub_trace.track_manage.shape4out.len = tempLen;
  sub_trace.track_manage.shape4out.wid = tempWid;
  sub_trace.track_manage.shape4out.height = sub_trace.X_(iBoxHeight);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void HmObjectTracker::track_simple_classifity() {
  // 新目标分类逻辑：根据模型检出的类型和置信度进行简单处理
  for (auto &trace : track_list) {
    if (trace.matched_info.matched_det_list.size() > 0)  // 本周期
    {
      auto &trace_type_manager = trace.track_manage.type_manage;
      const box_t &det_box = trace.matched_info.matched_det_list.at(0);

      if (det_box.type == trace_type_manager.type) {
        trace_type_manager.change_type_count = 0;
        if (det_box.score > trace_type_manager.score) {
          trace_type_manager.score +=
              (det_box.score - trace_type_manager.score) * 0.8;
        } else {
          trace_type_manager.score +=
              (det_box.score - trace_type_manager.score) * 0.5;
        }
      } else {
        trace_type_manager.change_type_count++;

        if (det_box.type != trace_type_manager.type_new) {
          trace_type_manager.change_type_count = 0;
        }

        trace_type_manager.type_new = det_box.type;
        trace_type_manager.score *= 0.8;

        // 连续N（3）次，进行目标类型切换
        if (trace_type_manager.change_type_count >= 3) {
          trace_type_manager.type = det_box.type;
          trace_type_manager.score = det_box.score * 0.8;
        }
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {double} x1
 * @param {double} y1
 * @param {double} line_k
 * @param {double} x2
 * @param {double} y2
 * @return {*}
 */
double HmObjectTracker::point2line_distance(double x1, double y1, double line_k,
                                            double x2, double y2) {
  return (fabs(line_k * y2 - x2 + (x1 - line_k * y2)) /
          (sqrt(line_k * line_k + 1.0)));
}

PERCEPTION_REGISTER_MULTITARGET_TRACKER(HmObjectTracker);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo