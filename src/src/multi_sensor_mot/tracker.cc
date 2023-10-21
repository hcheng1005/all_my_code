
#include "tracker.h"

#include <ctime>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <string>

#include "hungarian.h"
#include "visualizer.h"

namespace {

static uint64_t output_filename_timestamp = 0;
std::string GetOutputFilename(const std::string& scene_name) {
  if (output_filename_timestamp == 0) {
    time_t nowtime;
    time(&nowtime);
    output_filename_timestamp = static_cast<uint64_t>(nowtime);
  }
  const std::string output_filename =
      "../results/" + scene_name + "_results_" +
      std::to_string(output_filename_timestamp) + ".dat";
  return output_filename;
}

static constexpr double kTrackLidarDetectionDistanceGating = 3.0;
double GetTrackLidarDetectionDistance(const LidarDetection& detection,
                                      const Track& track) {
  const Vec3d error = detection.position() - track.position();
  return error.Length();
}

static constexpr double kTrackRadarDetectionDistanceGating = 3.5;
double GetTrackRadarDetectionDistance(const RadarDetection& detection,
                                      const Track& track) {
  const Vec2d pos_error = detection.position() - track.position().xy();
  const Vec2d vel_error = detection.velocity() - track.velocity().xy();
  static constexpr double pos_weight = 0.7;
  static constexpr double vel_weight = 0.3;
  return pos_weight * pos_error.Length() + vel_weight * vel_error.Length();
}

}  // namespace

void Tracker::Run(const LidarFrame& lidar_frame) {
  frame_index_ = lidar_frame.index();
  frame_timestamp_ = lidar_frame.timestamp();
  std::cout << "\nLidar frame_index: " << lidar_frame.index()
            << " detections_num: " << lidar_frame.detections().size()
            << " timestamp: " << std::fixed << std::setw(16)
            << std::setprecision(6) << lidar_frame.timestamp() << std::endl;

  PredictTracks(frame_timestamp_);

  std::vector<std::pair<uint32_t, uint32_t>> association_pairs;
  std::vector<uint32_t> unassociated_track_indices;
  std::vector<uint32_t> unassociated_detection_indices;
  DataAssociation(lidar_frame, &association_pairs, &unassociated_track_indices,
                  &unassociated_detection_indices);

  UpdateTracks(lidar_frame, association_pairs);

  ManagementTracks(lidar_frame, unassociated_track_indices,
                   unassociated_detection_indices);
}

void Tracker::Run(const RadarFrame& radar_frame) {
  frame_index_ = radar_frame.index();
  frame_timestamp_ = radar_frame.timestamp();
  std::cout << "\nRadar frame_index: " << radar_frame.index()
            << " detections_num: " << radar_frame.detections().size()
            << " timestamp: " << std::fixed << std::setw(16)
            << std::setprecision(6) << radar_frame.timestamp() << std::endl;

  PredictTracks(frame_timestamp_);

  std::vector<std::pair<uint32_t, uint32_t>> association_pairs;
  std::vector<uint32_t> unassociated_track_indices;
  std::vector<uint32_t> unassociated_detection_indices;
  DataAssociation(radar_frame, &association_pairs, &unassociated_track_indices,
                  &unassociated_detection_indices);

  UpdateTracks(radar_frame, association_pairs);
}

void Tracker::Run(const CameraFrame& camera_frame) {
  frame_index_ = camera_frame.index();
  frame_timestamp_ = camera_frame.timestamp();
  std::cout << "\nCamera frame: index: " << camera_frame.index()
            << " detections_num: " << camera_frame.detections().size()
            << " timestamp: " << std::fixed << std::setw(16)
            << std::setprecision(6) << camera_frame.timestamp() << std::endl;

  PredictTracks(frame_timestamp_);

  std::vector<std::pair<uint32_t, uint32_t>> association_pairs;
  std::vector<uint32_t> unassociated_track_indices;
  std::vector<uint32_t> unassociated_detection_indices;
  DataAssociation(camera_frame, &association_pairs, &unassociated_track_indices,
                  &unassociated_detection_indices);

  UpdateTracks(camera_frame, association_pairs);

  Visualizer(camera_frame, tracks_, scene_name_);
}

void Tracker::PredictTracks(const double& timestamp) {
  for (Track& track : tracks_) {
    track.Predict(timestamp);
  }
}

void Tracker::DataAssociation(
    const LidarFrame& lidar_frame,
    std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
    std::vector<uint32_t>* unassociated_track_indices,
    std::vector<uint32_t>* unassociated_detection_indices) {
  if (association_pairs == nullptr || unassociated_track_indices == nullptr ||
      unassociated_detection_indices == nullptr) {
    return;
  }

  if (lidar_frame.detections().empty()) {
    std::cout << "lidar_frame is empty! index: " << lidar_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << lidar_frame.timestamp() << std::endl;
    return;
  }

  if (tracks_.empty()) {
    std::cout << "tracks is empty! lidar_frame index: " << lidar_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << lidar_frame.timestamp() << std::endl;
    unassociated_detection_indices->resize(lidar_frame.detections().size());
    std::iota(unassociated_detection_indices->begin(),
              unassociated_detection_indices->end(), 0);
    return;
  }

  const std::vector<LidarDetection>& detections = lidar_frame.detections();
  std::vector<bool> is_track_associated(tracks_.size(), false);
  for (int i = 0; i < detections.size(); ++i) {
    double min_distance = kTrackLidarDetectionDistanceGating;
    int nearest_track_index = -1;
    for (int j = 0; j < tracks_.size(); ++j) {
      if (is_track_associated[j]) continue;
      const double distance =
          GetTrackLidarDetectionDistance(detections[i], tracks_[j]);
      if (distance < min_distance) {
        min_distance = distance;
        nearest_track_index = j;
      }
    }
    if (nearest_track_index > 0) {
      association_pairs->emplace_back(nearest_track_index, i);
      is_track_associated[nearest_track_index] = true;
    } else {
      unassociated_detection_indices->push_back(i);
    }
  }

  for (int i = 0; i < is_track_associated.size(); ++i) {
    if (!is_track_associated[i]) {
      unassociated_track_indices->push_back(i);
    }
  }

  std::cout << "Data Association for Lidar Frame: index: "
            << lidar_frame.index() << " timestamp: " << std::fixed
            << std::setw(16) << std::setprecision(6) << lidar_frame.timestamp()
            << std::endl;
  std::cout << "association_pairs: " << association_pairs->size()
            << " unassociated_track_indices: "
            << unassociated_track_indices->size()
            << " unassociated_track_indices: "
            << unassociated_detection_indices->size() << std::endl;
}

void Tracker::DataAssociation(
    const RadarFrame& radar_frame,
    std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
    std::vector<uint32_t>* unassociated_track_indices,
    std::vector<uint32_t>* unassociated_detection_indices) {
  if (association_pairs == nullptr || unassociated_track_indices == nullptr ||
      unassociated_detection_indices == nullptr) {
    return;
  }

  if (radar_frame.detections().empty()) {
    std::cout << "radar_frame is empty! index: " << radar_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << radar_frame.timestamp() << std::endl;
    return;
  }

  if (tracks_.empty()) {
    std::cout << "tracks is empty! radar_frame index: " << radar_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << radar_frame.timestamp() << std::endl;
    unassociated_detection_indices->resize(radar_frame.detections().size());
    std::iota(unassociated_detection_indices->begin(),
              unassociated_detection_indices->end(), 0);
    return;
  }

  const std::vector<RadarDetection>& detections = radar_frame.detections();
  std::vector<bool> is_track_associated(tracks_.size(), false);
  for (int i = 0; i < detections.size(); ++i) {
    double min_distance = kTrackRadarDetectionDistanceGating;
    int nearest_track_index = -1;
    for (int j = 0; j < tracks_.size(); ++j) {
      if (is_track_associated[j]) continue;
      const double distance =
          GetTrackRadarDetectionDistance(detections[i], tracks_[j]);
      if (distance < min_distance) {
        min_distance = distance;
        nearest_track_index = j;
      }
    }
    if (nearest_track_index > 0) {
      association_pairs->emplace_back(nearest_track_index, i);
      is_track_associated[nearest_track_index] = true;
    } else {
      unassociated_detection_indices->push_back(i);
    }
  }

  for (int i = 0; i < is_track_associated.size(); ++i) {
    if (!is_track_associated[i]) {
      unassociated_track_indices->push_back(i);
    }
  }

  std::cout << "Data Association for Lidar Frame: index: "
            << radar_frame.index() << " timestamp: " << std::fixed
            << std::setw(16) << std::setprecision(6) << radar_frame.timestamp()
            << std::endl;
  std::cout << "association_pairs: " << association_pairs->size()
            << " unassociated_track_indices: "
            << unassociated_track_indices->size()
            << " unassociated_track_indices: "
            << unassociated_detection_indices->size() << std::endl;
}

void Tracker::DataAssociation(
    const CameraFrame& camera_frame,
    std::vector<std::pair<uint32_t, uint32_t>>* association_pairs,
    std::vector<uint32_t>* unassociated_track_indices,
    std::vector<uint32_t>* unassociated_detection_indices) {
  if (association_pairs == nullptr || unassociated_track_indices == nullptr ||
      unassociated_detection_indices == nullptr) {
    return;
  }

  if (camera_frame.detections().empty()) {
    std::cout << "camera_frame is empty! index: " << camera_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << camera_frame.timestamp() << std::endl;
    return;
  }

  if (tracks_.empty()) {
    std::cout << "tracks is empty! camera_frame index: " << camera_frame.index()
              << " timestamp: " << std::fixed << std::setw(16)
              << std::setprecision(6) << camera_frame.timestamp() << std::endl;
    unassociated_detection_indices->resize(camera_frame.detections().size());
    std::iota(unassociated_detection_indices->begin(),
              unassociated_detection_indices->end(), 0);
    return;
  }

  const std::vector<CameraDetection>& detections = camera_frame.detections();
  std::vector<std::vector<double>> iou_matrix(tracks_.size());
  for (int i = 0; i < tracks_.size(); ++i) {
    iou_matrix[i].resize(detections.size());
    AABox track_bbox;
    if (!tracks_[i].GetProjectionOnImage(camera_frame.world_to_vehicle(),
                                         &track_bbox)) {
      continue;
    }
    const int track_area = track_bbox.Area();
    for (int j = 0; j < detections.size(); ++j) {
      const double intersection =
          static_cast<double>(track_bbox.Intersection(detections[j].bbox()));
      const double union_area = static_cast<double>(
          track_area + detections[j].bbox().Area() - intersection);
      iou_matrix[i][j] = intersection / union_area;
    }
  }

  static constexpr double kMinIoUGating = 0.2;
  HungarianMaximize(iou_matrix, kMinIoUGating, association_pairs,
                    unassociated_track_indices, unassociated_detection_indices);

  std::cout << "Data Association for Camera Frame: index: "
            << camera_frame.index() << " timestamp: " << std::fixed
            << std::setw(16) << std::setprecision(6) << camera_frame.timestamp()
            << std::endl;
  std::cout << "association_pairs: " << association_pairs->size()
            << " unassociated_track_indices: "
            << unassociated_track_indices->size()
            << " unassociated_track_indices: "
            << unassociated_detection_indices->size() << std::endl;
}

void Tracker::UpdateTracks(
    const LidarFrame& lidar_frame,
    const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs) {
  const std::vector<LidarDetection>& detections = lidar_frame.detections();
  for (const auto& pair : association_pairs) {
    const int track_index = pair.first;
    const int detection_index = pair.second;
    tracks_[track_index].Update(detections[detection_index]);
  }
}

void Tracker::UpdateTracks(
    const RadarFrame& radar_frame,
    const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs) {
  const std::vector<RadarDetection>& detections = radar_frame.detections();
  for (const auto& pair : association_pairs) {
    const int track_index = pair.first;
    const int detection_index = pair.second;
    tracks_[track_index].Update(detections[detection_index]);
  }
}

void Tracker::UpdateTracks(
    const CameraFrame& camera_frame,
    const std::vector<std::pair<uint32_t, uint32_t>>& association_pairs) {
  const std::vector<CameraDetection>& detections = camera_frame.detections();
  for (const auto& pair : association_pairs) {
    const int track_index = pair.first;
    const int detection_index = pair.second;
    tracks_[track_index].Update(detections[detection_index]);
  }
}

void Tracker::ManagementTracks(
    const LidarFrame& lidar_frame,
    const std::vector<uint32_t>& unassociated_track_indices,
    const std::vector<uint32_t>& unassociated_detection_indices) {
  // Create new tracks
  const std::vector<LidarDetection>& detections = lidar_frame.detections();
  for (const int index : unassociated_detection_indices) {
    tracks_.emplace_back(detections[index]);
  }

  // Remove lost tracks
  auto iter = tracks_.begin();
  while (iter != tracks_.end()) {
    if (iter->IsLost()) {
      iter = tracks_.erase(iter);
    } else {
      ++iter;
    }
  }
}

std::vector<Track> Tracker::PublishTracks() {
  std::vector<Track> published_tracks;
  for (const Track& track : tracks_) {
    if (track.IsConfirmed()) {
      published_tracks.push_back(track);
    }
  }

  const std::string output_filename = GetOutputFilename(scene_name_);
  std::ofstream output_file(output_filename, std::ios::out | std::ios::app);

  if (!output_file.is_open()) {
    std::cout << "Fail to open " << output_filename << std::endl;
    return published_tracks;
  }

  output_file << frame_index_ << " " << published_tracks.size() << " "
              << std::fixed << std::setw(16) << std::setprecision(6)
              << frame_timestamp_ << std::endl;

  if (!published_tracks.empty()) {
    for (int i = 0; i < tracks_.size(); ++i) {
      if (tracks_[i].IsConfirmed()) {
        output_file << tracks_[i].id() << "  " << tracks_[i].position().x()
                    << " " << tracks_[i].position().y() << " "
                    << tracks_[i].position().z() << " "
                    << tracks_[i].velocity().x() << " "
                    << tracks_[i].velocity().y() << " "
                    << tracks_[i].velocity().z() << " "
                    << tracks_[i].acceleration().x() << " "
                    << tracks_[i].acceleration().y() << " "
                    << tracks_[i].acceleration().z() << " "
                    << tracks_[i].size().x() << " " << tracks_[i].size().y()
                    << " " << tracks_[i].size().z() << " " << tracks_[i].yaw()
                    << " " << tracks_[i].yaw_rate() << std::endl;
      }
    }
  }

  output_file << std::endl;
  output_file.close();
  std::cout << "Publish " << published_tracks.size() << " tracks to "
            << output_filename << " at timestamp: " << std::fixed
            << std::setw(16) << std::setprecision(6) << frame_timestamp_
            << std::endl;
  return published_tracks;
}