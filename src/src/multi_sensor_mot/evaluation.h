#pragma once

#include <vector>

#include "hungarian.h"
#include "track.h"

namespace {

static constexpr double kMaximumDistanceGating = 2.0;

double CalculateDistance(const Track& t1, const Track& t2) {
  const Vec3d distance_vector = t1.position() - t2.position();
  return distance_vector.Length();
}

std::vector<std::vector<double>> GetDistanceMatrix(
    const std::vector<Track>& gt_tracks,
    const std::vector<Track>& published_tracks) {
  std::vector<std::vector<double>> distances;
  for (const Track& gt_track : gt_tracks) {
    std::vector<double> distance;
    for (const Track& pub_track : published_tracks) {
      distance.push_back(CalculateDistance(gt_track, pub_track));
    }
    distances.push_back(distance);
  }
  return distances;
}

std::vector<std::pair<uint32_t, uint32_t>> InheritCorrespondence(
    const std::vector<std::pair<uint32_t, uint32_t>>& last_correspondences,
    std::vector<Track>* gt_tracks, std::vector<Track>* published_tracks) {
  std::vector<std::pair<uint32_t, uint32_t>> correspondences;
  for (const auto& pair : last_correspondences) {
    const uint32_t& last_gt_track_id = pair.first;
    const uint32_t& last_pub_track_id = pair.second;
    auto current_gt_track_iter =
        std::find_if(gt_tracks->begin(), gt_tracks->end(),
                     [&last_gt_track_id](const Track& track) -> bool {
                       return track.id() == last_gt_track_id;
                     });
    auto current_pub_track_iter =
        std::find_if(published_tracks->begin(), published_tracks->end(),
                     [&last_pub_track_id](const Track& track) -> bool {
                       return track.id() == last_pub_track_id;
                     });
    if (current_gt_track_iter == gt_tracks->end() ||
        current_pub_track_iter == published_tracks->end()) {
      continue;
    }
    if (CalculateDistance(*current_gt_track_iter, *current_pub_track_iter) <
        kMaximumDistanceGating) {
      correspondences.emplace_back(last_gt_track_id, last_pub_track_id);
      gt_tracks->erase(current_gt_track_iter);
      published_tracks->erase(current_pub_track_iter);
    }
  }
  return correspondences;
}

}  // namespace

double PerformanceEvaluation(
    const std::vector<std::vector<Track>>& published_tracks_list,
    const std::vector<std::vector<Track>>& gt_tracks_list) {
  if (published_tracks_list.size() != gt_tracks_list.size()) {
    std::cout << "Frames number is wrong! published_tracks_list: "
              << published_tracks_list.size()
              << " gt_tracks_list: " << gt_tracks_list.size() << std::endl;
    return 0.0;
  }

  std::cout << "\nPerformance Evaluation Results" << std::endl;

  int ground_truth_num = 0;
  int miss_detection_num = 0;
  int false_positive_num = 0;
  int mismatch_num = 0;

  const int frame_num = gt_tracks_list.size();
  std::vector<std::pair<uint32_t, uint32_t>> last_correspondences;
  for (int k = 0; k < frame_num; ++k) {
    ground_truth_num += gt_tracks_list[k].size();
    std::vector<Track> gt_tracks = gt_tracks_list[k];
    std::vector<Track> pub_tracks = published_tracks_list[k];
    std::vector<std::pair<uint32_t, uint32_t>> current_correspondences =
        InheritCorrespondence(last_correspondences, &gt_tracks, &pub_tracks);

    if (!gt_tracks.empty() && !pub_tracks.empty()) {
      // Calculate new correspondences
      const std::vector<std::vector<double>> distances =
          GetDistanceMatrix(gt_tracks, pub_tracks);
      std::vector<std::pair<uint32_t, uint32_t>> associations;
      std::vector<uint32_t> unassociated_rows;
      std::vector<uint32_t> unassociated_cols;
      HungarianMinimize(distances, kMaximumDistanceGating, &associations,
                        &unassociated_rows, &unassociated_cols);

      int mismatch_cnt = 0;
      int miss_detections_cnt = unassociated_rows.size();
      int false_positives_cnt = unassociated_cols.size();
      int pair_cnt = 0;
      for (const auto& pair : associations) {
        if (distances[pair.first][pair.second] < kMaximumDistanceGating) {
          const uint32_t gt_track_id = gt_tracks[pair.first].id();
          const uint32_t pub_track_id = pub_tracks[pair.second].id();
          current_correspondences.emplace_back(gt_track_id, pub_track_id);
          auto last_correspondence_iter = std::find_if(
              last_correspondences.begin(), last_correspondences.end(),
              [&gt_track_id](const std::pair<uint32_t, uint32_t>& id_pair)
                  -> bool { return id_pair.first == gt_track_id; });
          if (last_correspondence_iter != last_correspondences.end() &&
              last_correspondence_iter->second != pub_track_id) {
            ++mismatch_cnt;
          }
        } else {
          ++miss_detections_cnt;
          ++false_positives_cnt;
        }
      }

      miss_detection_num += miss_detections_cnt;
      false_positive_num += false_positives_cnt;
      mismatch_num += mismatch_cnt;
    } else if (gt_tracks.empty() && !pub_tracks.empty()) {
      false_positive_num += pub_tracks.size();
    } else if (pub_tracks.empty() && !gt_tracks.empty()) {
      miss_detection_num += gt_tracks.size();
    }

    last_correspondences = current_correspondences;
  }

  const double miss_detection_rate = static_cast<double>(miss_detection_num) /
                                     static_cast<double>(ground_truth_num);
  const double false_positive_rate = static_cast<double>(false_positive_num) /
                                     static_cast<double>(ground_truth_num);
  const double mismatch_rate =
      static_cast<double>(mismatch_num) / static_cast<double>(ground_truth_num);
  const double mota =
      1 - miss_detection_rate - false_positive_rate - mismatch_rate;

  std::cout << "miss_detection_num: " << miss_detection_num
            << " false_positive_num: " << false_positive_num
            << " mismatch_num: " << mismatch_num
            << " ground_truth_num: " << ground_truth_num << std::endl;
  std::cout << "miss_detection_rate: " << miss_detection_rate
            << " false_positive_rate: " << false_positive_rate
            << " mismatch_rate: " << mismatch_rate << "  MOTA: " << mota
            << std::endl;

  return mota;
}
