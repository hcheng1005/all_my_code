#pragma once

#include <map>
#include <vector>

#include "camera_detection.h"
#include "kalman_filter.h"
#include "lidar_detection.h"
#include "math.h"
#include "radar_detection.h"

class Track {
 public:
  Track() = default;
  ~Track() = default;

  Track(const LidarDetection& detection,
        const Category category = Category::kUnknwon);

  void SetId(const uint32_t id) { id_ = id; }

  uint32_t id() const { return id_; }
  double created_timestamp() const { return created_timestamp_; }
  double timestamp() const { return timestamp_; }
  double last_lidar_update_timestamp() const {
    return last_lidar_update_timestamp_;
  }
  double last_camera_update_timestamp() const {
    return last_camera_update_timestamp_;
  }
  double Age() const { return timestamp_ - created_timestamp_; }

  Category category() const { return category_; }

  Vec3d position() const { return position_; }
  Vec3d velocity() const { return velocity_; }
  Vec3d acceleration() const { return acceleration_; }
  Vec3d size() const { return size_; }
  double length() const { return size_.x(); }
  double width() const { return size_.y(); }
  double height() const { return size_.z(); }
  double yaw() const { return yaw_; }
  double yaw_rate() const { return yaw_rate_; }

  void Predict(const double timestamp);
  void Update(const LidarDetection& detection);
  void Update(const RadarDetection& detection);
  void Update(const CameraDetection& detection);

  bool IsLost() const;
  bool IsConfirmed() const;

  std::vector<Vec2d> GetCorners() const;

  bool GetProjectionOnImage(const Transformation2d& world_to_vehicle,
                            AABox* bbox) const;

  static void ResetTrackId() { next_track_id_ = 1; }

  static void SetCameraProjection(const Vec3d& translation, double rotation[4],
                                  double intrinsic[4]) {
    camera_projection_ = CameraProjection(translation, rotation, intrinsic);
  }

 private:
  uint32_t id_;
  double created_timestamp_;
  double timestamp_;
  double last_lidar_update_timestamp_;
  double last_radar_update_timestamp_;
  double last_camera_update_timestamp_;
  Vec3d position_;
  Vec3d velocity_;
  Vec3d acceleration_;
  KalmanFilter<9> motion_kf_;
  Vec3d size_;
  double yaw_;
  double yaw_rate_;
  KalmanFilter<2> yaw_kf_;

  Category category_;
  std::map<Category, int> category_vote_;

  int associated_lidar_detections_cnt_;
  int associated_radar_detections_cnt_;
  int associated_camera_detections_cnt_;

  static uint32_t next_track_id_;
  static CameraProjection camera_projection_;
};