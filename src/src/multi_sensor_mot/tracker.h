#pragma once

#include <vector>

#include "camera_detection.h"
#include "lidar_detection.h"
#include "radar_detection.h"
#include "track.h"

class Tracker {
 public:
  Tracker() = default;
  ~Tracker() = default;

  Tracker(const std::string& scene_name) : scene_name_(scene_name) {}

  void Run(const LidarFrame& lidar_frame);

  void Run(const RadarFrame& radar_frame);

  void Run(const CameraFrame& camera_frame);

  void PredictTracks(const double& timestamp);

  void DataAssociation(
      const LidarFrame& lidar_frame,
      std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
      std::vector<uint32_t>* unassociated_track_indices,
      std::vector<uint32_t>* unassociated_detection_indices);

  void DataAssociation(
      const RadarFrame& radar_frame,
      std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
      std::vector<uint32_t>* unassociated_track_indices,
      std::vector<uint32_t>* unassociated_detection_indices);

  void DataAssociation(
      const CameraFrame& camera_frame,
      std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
      std::vector<uint32_t>* unassociated_track_indices,
      std::vector<uint32_t>* unassociated_detection_indices);

  void UpdateTracks(
      const LidarFrame& lidar_frame,
      const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs);

  void UpdateTracks(
      const RadarFrame& radar_frame,
      const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs);

  void UpdateTracks(
      const CameraFrame& camera_frame,
      const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs);

  void ManagementTracks(
      const LidarFrame& lidar_frame,
      const std::vector<uint32_t>& unassociated_track_indices,
      const std::vector<uint32_t>& unassociated_detection_indices);

  std::vector<Track> PublishTracks();

 private:
  std::vector<Track> tracks_;
  const std::string scene_name_;
  int frame_index_;
  double frame_timestamp_;
};
