
#include "track.h"

namespace
{

  static constexpr double Pi = 3.14159265358979323846;

  static constexpr double kInitPositionVariance = 4.0;
  static constexpr double kInitVelocityVariance = 9.0;
  static constexpr double kInitAccelerationVariance = 2.0;
  static constexpr double kInitYawVariance = 0.01;
  static constexpr double kInitYawRateVariance = 0.01;

  static constexpr double kPredictPositionVariance = 0.25;
  static constexpr double kPredictVelocityVariance = 0.09;
  static constexpr double kPredictAccelerationVariance = 0.04;
  static constexpr double kPredictYawVariance = 0.01;
  static constexpr double kPredictYawRateVariance = 0.01;

  static constexpr double kLidarPositionNoiseVariance = 0.25;
  static constexpr double kLidarYawNoiseVariance = 0.01;

  static constexpr double kRadarPositionNoiseVariance = 1.0;
  static constexpr double kRadarVelocityNoiseVariance = 0.25;

  static constexpr double kSmoothFilterCoefficient = 0.7;

  static constexpr double kReliableDetectionRange = 50.0; // meter

  Vec2d RotateByTheta(const Vec2d p, const double theta)
  {
    return Vec2d(p.x() * std::cos(theta) + p.y() * std::sin(theta),
                 p.y() * std::cos(theta) - p.x() * std::sin(theta));
  }

} // namespace

uint32_t Track::next_track_id_ = 1;
CameraProjection Track::camera_projection_;

Track::Track(const LidarDetection &detection, const Category category)
    : id_(next_track_id_++),
      created_timestamp_(detection.timestamp()),
      timestamp_(detection.timestamp()),
      last_lidar_update_timestamp_(detection.timestamp()),
      last_radar_update_timestamp_(0.0),
      last_camera_update_timestamp_(0.0),
      category_(category),
      position_(detection.position()),
      size_(detection.size()),
      yaw_(detection.yaw()),
      associated_lidar_detections_cnt_(1),
      associated_radar_detections_cnt_(0),
      associated_camera_detections_cnt_(0)
{
  velocity_ = Vec3d();
  acceleration_ = Vec3d();
  Eigen::Matrix<double, 9, 1> state; // = Eigen::Matrix<double, 9, 1>();
  state << position_.x(), position_.y(), position_.z(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::Matrix<double, 9, 9> covariance = Eigen::Matrix<double, 9, 9>::Zero();
  covariance(0, 0) = covariance(1, 1) = covariance(2, 2) =
      kInitPositionVariance;
  covariance(3, 3) = covariance(4, 4) = covariance(5, 5) =
      kInitVelocityVariance;
  covariance(6, 6) = covariance(7, 7) = covariance(8, 8) =
      kInitAccelerationVariance;

  motion_kf_.set_state(state);
  motion_kf_.set_covariance(covariance);

  yaw_rate_ = 0.0;
  const Eigen::Matrix<double, 2, 1> &yaw_state{yaw_, yaw_rate_};
  Eigen::Matrix<double, 2, 2> yaw_covariance =
      Eigen::Matrix<double, 2, 2>::Zero();
  yaw_covariance(0, 0) = kInitYawVariance;
  yaw_covariance(1, 1) = kInitYawRateVariance;

  yaw_kf_.set_state(yaw_state);
  yaw_kf_.set_covariance(yaw_covariance);

  category_vote_.insert(std::make_pair(Category::kUnknwon, 0));
  category_vote_.insert(std::make_pair(Category::kPerson, 0));
  category_vote_.insert(std::make_pair(Category::kVehicle, 0));
}

void Track::Predict(const double timestamp)
{
  if (timestamp <= timestamp_)
    return;
  const double dt = timestamp - timestamp_;
  const double dt2 = dt * dt;

  timestamp_ = timestamp;

  Eigen::Matrix<double, 9, 9> transition_matrix;
  transition_matrix << 1, 0, 0, dt, 0, 0, dt2, 0, 0, 0, 1, 0, 0, dt, 0, 0, dt2,
      0, 0, 0, 1, 0, 0, dt, 0, 0, dt2, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0,
      1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

  static constexpr double kStandardTimeInterval = 0.5; // second
  const double time_scale = dt / kStandardTimeInterval;

  Eigen::Matrix<double, 9, 9> predict_noise_cov =
      Eigen::Matrix<double, 9, 9>::Zero();
  predict_noise_cov(0, 0) = predict_noise_cov(1, 1) = predict_noise_cov(2, 2) =
      time_scale * kPredictPositionVariance;
  predict_noise_cov(3, 3) = predict_noise_cov(4, 4) = predict_noise_cov(5, 5) =
      time_scale * kPredictVelocityVariance;
  predict_noise_cov(6, 6) = predict_noise_cov(7, 7) = predict_noise_cov(8, 8) =
      time_scale * kPredictAccelerationVariance;

  motion_kf_.Predict(transition_matrix, predict_noise_cov);
  position_ = Vec3d(motion_kf_.state()[0], motion_kf_.state()[1],
                    motion_kf_.state()[2]);
  velocity_ = Vec3d(motion_kf_.state()[3], motion_kf_.state()[4],
                    motion_kf_.state()[5]);
  acceleration_ = Vec3d(motion_kf_.state()[6], motion_kf_.state()[7],
                        motion_kf_.state()[8]);

  Eigen::Matrix<double, 2, 2> yaw_transition_matrix; //{{1, dt}, {0, 1}};
  yaw_transition_matrix << 1, dt, 0, 1;

  Eigen::Matrix<double, 2, 2> yaw_predict_noise_cov; //{{time_scale * kPredictYawVariance, 0}, {0, time_scale * kPredictYawRateVariance}};
  yaw_predict_noise_cov << time_scale * kPredictYawVariance, 0, 0, time_scale * kPredictYawRateVariance;

  yaw_kf_.Predict(yaw_transition_matrix, yaw_predict_noise_cov);
  yaw_ = yaw_kf_.state()[0];
  yaw_rate_ = yaw_kf_.state()[1];
}

void Track::Update(const LidarDetection &detection)
{
  const Eigen::Matrix<double, 3, 1> measurement{detection.position().x(),
                                                detection.position().y(),
                                                detection.position().z()};
  const double noise_scale =
      detection.distance_to_ego() < kReliableDetectionRange ? 1.0 : 2.0;
  const Eigen::Matrix<double, 3, 3> measure_noise_cov =
      noise_scale * kLidarPositionNoiseVariance *
      Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 9> measure_matrix =
      Eigen::Matrix<double, 3, 9>::Zero();
  measure_matrix.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
  motion_kf_.Update<3>(measurement, measure_noise_cov, measure_matrix);

  position_ = Vec3d(motion_kf_.state()[0], motion_kf_.state()[1],
                    motion_kf_.state()[2]);
  velocity_ = Vec3d(motion_kf_.state()[3], motion_kf_.state()[4],
                    motion_kf_.state()[5]);
  acceleration_ = Vec3d(motion_kf_.state()[6], motion_kf_.state()[7],
                        motion_kf_.state()[8]);

  const Eigen::Matrix<double, 1, 1> yaw_measurement{detection.yaw()};
  const Eigen::Matrix<double, 1, 1> yaw_measure_noise_cov{
      noise_scale * kLidarYawNoiseVariance};
  Eigen::Matrix<double, 1, 2> yaw_measure_matrix{1, 0};
  yaw_kf_.Update<1>(yaw_measurement, yaw_measure_noise_cov, yaw_measure_matrix);
  yaw_ = yaw_kf_.state()[0];
  if (yaw_ > Pi)
    yaw_ -= 2 * Pi;
  else if (yaw_ < -Pi)
    yaw_ += 2 * Pi;
  yaw_rate_ = yaw_kf_.state()[1];

  size_ = size_ * kSmoothFilterCoefficient +
          detection.size() * (1 - kSmoothFilterCoefficient);

  last_lidar_update_timestamp_ = detection.timestamp();
  ++associated_lidar_detections_cnt_;
}

void Track::Update(const RadarDetection &detection)
{
  const Eigen::Matrix<double, 4, 1> measurement{
      detection.position().x(), detection.position().y(),
      detection.velocity().x(), detection.velocity().y()};
  const double noise_scale =
      detection.distance_to_ego() < kReliableDetectionRange ? 1.0 : 2.0;
  Eigen::Matrix<double, 4, 4> measure_noise_cov =
      Eigen::Matrix<double, 4, 4>::Zero();
  measure_noise_cov.block(0, 0, 2, 2) = noise_scale *
                                        kRadarPositionNoiseVariance *
                                        Eigen::Matrix<double, 2, 2>::Identity();
  measure_noise_cov.block(2, 2, 2, 2) = noise_scale *
                                        kRadarVelocityNoiseVariance *
                                        Eigen::Matrix<double, 2, 2>::Identity();

  Eigen::Matrix<double, 4, 9> measure_matrix =
      Eigen::Matrix<double, 4, 9>::Zero();
  measure_matrix.block(0, 0, 2, 2) = Eigen::Matrix<double, 2, 2>::Identity();
  measure_matrix.block(2, 3, 2, 2) = Eigen::Matrix<double, 2, 2>::Identity();
  motion_kf_.Update<4>(measurement, measure_noise_cov, measure_matrix);

  position_ = Vec3d(motion_kf_.state()[0], motion_kf_.state()[1],
                    motion_kf_.state()[2]);
  velocity_ = Vec3d(motion_kf_.state()[3], motion_kf_.state()[4],
                    motion_kf_.state()[5]);
  acceleration_ = Vec3d(motion_kf_.state()[6], motion_kf_.state()[7],
                        motion_kf_.state()[8]);

  last_radar_update_timestamp_ = detection.timestamp();
  ++associated_radar_detections_cnt_;
}

void Track::Update(const CameraDetection &detection)
{
  last_camera_update_timestamp_ = detection.timestamp();
  ++associated_camera_detections_cnt_;

  if (++category_vote_[detection.category()] > category_vote_[category_])
  {
    category_ = detection.category();
  }
}

bool Track::IsLost() const
{
  const double kTrackLostTimeGating = IsConfirmed() ? 1.5 : 0.1; // second
  return timestamp_ - last_lidar_update_timestamp_ > kTrackLostTimeGating;
}

bool Track::IsConfirmed() const
{
  static constexpr int kMinObservationTimes = 3;
  return associated_lidar_detections_cnt_ + associated_radar_detections_cnt_ +
             associated_camera_detections_cnt_ >=
         kMinObservationTimes;
}

std::vector<Vec2d> Track::GetCorners() const
{
  const double theta = -yaw_;
  const Vec2d center(position_.x(), position_.y());
  const Vec2d tl_shift(0.5 * size_.x(), 0.5 * size_.y());
  const Vec2d tr_shift(0.5 * size_.x(), -0.5 * size_.y());
  const Vec2d bl_shift(-0.5 * size_.x(), 0.5 * size_.y());
  const Vec2d br_shift(-0.5 * size_.x(), -0.5 * size_.y());
  const Vec2d top_left = center + RotateByTheta(tl_shift, theta);
  const Vec2d top_right = center + RotateByTheta(tr_shift, theta);
  const Vec2d bottom_left = center + RotateByTheta(bl_shift, theta);
  const Vec2d bottom_right = center + RotateByTheta(br_shift, theta);
  return std::vector<Vec2d>{top_left, bottom_left, bottom_right, top_right};
}

bool Track::GetProjectionOnImage(const Transformation2d &world_to_vehicle,
                                 AABox *bbox) const
{
  const std::vector<Vec2d> corners = GetCorners();
  int min_x = CameraProjection::kImageWidthPixel;
  int max_x = 0;
  int min_y = CameraProjection::kImageHeightPixel;
  int max_y = 0;
  for (int i = 0; i < 4; ++i)
  {
    const Vec2d corner_vehicle = world_to_vehicle.Transform(corners[i]);
    const Vec3d corner_vehicle_bottom(corner_vehicle, 0.);
    const Vec3d corner_vehicle_top(corner_vehicle, height());

    int u, v;
    if (camera_projection_.VehicleToImage(corner_vehicle_bottom, &u, &v))
    {
      min_x = std::min(min_x, u);
      max_x = std::max(max_x, u);
      min_y = std::min(min_y, v);
      max_y = std::max(max_y, v);
    }
    if (camera_projection_.VehicleToImage(corner_vehicle_top, &u, &v))
    {
      min_x = std::min(min_x, u);
      max_x = std::max(max_x, u);
      min_y = std::min(min_y, v);
      max_y = std::max(max_y, v);
    }
  }

  static constexpr int margin = 20;
  if (max_x > min_x + margin && max_y > min_y + margin)
  {
    *bbox = AABox(min_x, min_y, max_x - min_x, max_y - min_y);
    return true;
  }

  return false;
}