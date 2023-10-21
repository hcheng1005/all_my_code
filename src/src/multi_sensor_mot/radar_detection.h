#pragma once

#include <iostream>
#include <vector>

#include "math.h"

class RadarDetection {
 public:
  RadarDetection() = default;
  ~RadarDetection() = default;

  RadarDetection(const uint32_t id, const double timestamp, Vec2d position,
                 Vec2d velocity)
      : id_(id),
        timestamp_(timestamp),
        position_(position),
        velocity_(velocity),
        distance_to_ego_(-1.0) {}

  uint32_t id() const { return id_; }
  double timestamp() const { return timestamp_; }
  Vec2d position() const { return position_; }
  Vec2d velocity() const { return velocity_; }
  double distance_to_ego() const { return distance_to_ego_; }

  void GetDistanceToEgo(const Vec2d ego_position) {
    const Vec2d vector_to_ego = position_ - ego_position;
    distance_to_ego_ = vector_to_ego.Length();
  }

 private:
  uint32_t id_;
  double timestamp_;
  Vec2d position_;
  Vec2d velocity_;
  double distance_to_ego_;
};

class RadarFrame {
 public:
  RadarFrame() = default;
  ~RadarFrame() = default;

  RadarFrame(uint32_t index, double timestamp,
             const Transformation2d& world_to_vehicle,
             const std::vector<RadarDetection>& detections)
      : index_(index),
        timestamp_(timestamp),
        world_to_vehicle_(world_to_vehicle),
        detections_(detections) {}

  uint32_t index() const { return index_; }
  double timestamp() const { return timestamp_; }
  const Transformation2d& world_to_vehicle() const { return world_to_vehicle_; }
  const std::vector<RadarDetection>& detections() const { return detections_; }

 private:
  uint32_t index_;
  double timestamp_;
  Transformation2d world_to_vehicle_;
  std::vector<RadarDetection> detections_;
};
