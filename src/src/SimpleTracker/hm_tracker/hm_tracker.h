#pragma once

#include <string>

#include "modules/perception/lidar/common/lidar_frame.h"
#include "modules/perception/lidar/lib/interface/base_multi_target_tracker.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/iou.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/kalman.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/lshape.h"
#include "modules/perception/onboard/transform_wrapper/transform_vel.h"
#include "modules/perception/lidar/lib/map_manager/map_manager.h"
#include "modules/perception/common/graph/hungarian_optimizer.h"

namespace apollo {
namespace perception {
namespace lidar {

class HmObjectTracker : public BaseMultiTargetTracker {
 public:
  HmObjectTracker() = default;
  virtual ~HmObjectTracker() = default;

  bool Init(const MultiTargetTrackerInitOptions &options =
                MultiTargetTrackerInitOptions()) override;

  bool Track(const MultiTargetTrackerOptions &options,
             LidarFrame *frame) override;

  std::string Name() const override { return "HmObjectTracker"; }

 private:
  const bool USING_MODEL_DETECTION = true;
  bool DEBUG_SWITCH = false;
  bool OUT_DETECTION_OBJ = DEBUG_SWITCH;
  MapManager map_manager_;
  apollo::perception::common::HungarianOptimizer<float>* optimizer_ = nullptr;

  std::vector<box_t> trace_box_list;
  std::vector<box_t> det_box_list;
  std::vector<simple_tracker> track_list;
  std::unique_ptr<apollo::perception::onboard::transform_vel> transform_vel_;

  void lidar_tracking_proc(LidarFrame &frame, const double dt);
  void lidar_trace_predict(LidarFrame &frame, double dt);
  void creat_det_box(LidarFrame &frame, std::vector<box_t> &det_box_list);
  void creat_trace_box(std::vector<box_t> &trace_box_list);
  void match_trace_wiht_det(LidarFrame &frame,
                            std::vector<box_t> &trace_box_list,
                            std::vector<box_t> &det_box_list);
  void assignment(LidarFrame &frame,
                  const std::vector<greedy_match_info_t> &cost_matrix,
                  std::vector<box_t> &trace_box_list,
                  std::vector<box_t> &det_box_list);

  void assignment_used_for_model(LidarFrame &frame,
                  std::vector<greedy_match_info_t> &cost_matrix,
                  std::vector<box_t> &trace_box_list,
                  std::vector<box_t> &det_box_list);

  std::vector<greedy_match_info_t> build_cost_matrix(
      std::vector<box_t> &trace_box_list, std::vector<box_t> &det_box_list);

  static bool cost_compare(const greedy_match_info_t &c1,
                           const greedy_match_info_t &c2);

  void refitting_det_box(LidarFrame &frame, const Eigen::VectorXd trace_status,
                         std::vector<uint32_t> det_list,
                         std::vector<box_t> &det_box_list, box_t &new_det_box);

  void track_update_all();
  void track_managment(std::vector<box_t> &det_box_list);
  void track_simple_classifity();
  void trace_ComputeAbsVel(simple_tracker &trace, const double yawRate,
                           const double speed);
  void trace_UpdateMotionStatus(simple_tracker &trace);
  void trace_UpdateHeading(simple_tracker &sub_trace);

  bool recognise_dust(std::shared_ptr<LidarObject> &det);

  void compute_shape4out(LidarFrame &frame);
  double point2line_distance(double x1, double y1, double line_k, double x2,
                             double y2);
  void re_compute_shape(box_t &sub_det_box);
  void trans_obj2frame(LidarFrame &frame);
  bool detecObjIndoor(const box_t &det_box, const Eigen::Matrix4d lidar2world);
  std::string objInnerType(const inner_type_enum type);
  void output_detection_result(LidarFrame &frame);
  bool vehicle_status_update(LidarFrame &frame);
  std::string objMotionStatus(const base::MotionState &motion_status);
};  // class HmObjectTracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo