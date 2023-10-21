#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "math.h"

enum Category { kPerson = 0, kVehicle = 1, kUnknwon = 2 };

// Axis-Aligned Box
class AABox {
 public:
  AABox() = default;
  ~AABox() = default;

  AABox(int top_left_x, int top_left_y, int width, int height)
      : top_left_x_(top_left_x),
        top_left_y_(top_left_y),
        width_(width),
        height_(height) {}

  int top_left_x() const { return top_left_x_; };
  int top_left_y() const { return top_left_y_; };
  int width() const { return width_; };
  int height() const { return height_; };

  int Area() const { return width_ * height_; }
  int Intersection(const AABox& b) const {
    const int intersection_x =
        std::min(top_left_x_ + width_, b.top_left_x() + b.width()) -
        std::max(top_left_x_, b.top_left_x());
    const int intersection_y =
        std::min(top_left_y_ + height_, b.top_left_y() + b.height()) -
        std::max(top_left_y_, b.top_left_y());
    return intersection_x <= 0 || intersection_y <= 0
               ? 0
               : intersection_x * intersection_y;
  }

 private:
  int top_left_x_;
  int top_left_y_;
  int width_;
  int height_;
};

class CameraDetection {
 public:
  CameraDetection() = default;
  ~CameraDetection() = default;

  CameraDetection(uint32_t id, double timestamp, const int category,
                  const AABox& bbox)
      : id_(id),
        timestamp_(timestamp),
        category_(static_cast<Category>(category)),
        bbox_(bbox) {}

  uint32_t id() const { return id_; }
  double timestamp() const { return timestamp_; }
  Category category() const { return category_; }
  const AABox& bbox() const { return bbox_; }

 private:
  uint32_t id_;
  double timestamp_;
  Category category_;
  AABox bbox_;
};

class CameraFrame {
 public:
  CameraFrame() = default;
  ~CameraFrame() = default;

  CameraFrame(uint32_t index, double timestamp,
              const std::string& image_filename,
              const Transformation2d& world_to_vehicle,
              const std::vector<CameraDetection>& detections)
      : index_(index),
        timestamp_(timestamp),
        image_filename_(image_filename),
        world_to_vehicle_(world_to_vehicle),
        detections_(detections) {}

  uint32_t index() const { return index_; }
  double timestamp() const { return timestamp_; }
  std::string image_filename() const { return image_filename_; }
  const Transformation2d& world_to_vehicle() const { return world_to_vehicle_; }
  const std::vector<CameraDetection>& detections() const { return detections_; }

 private:
  uint32_t index_;
  double timestamp_;
  std::string image_filename_;
  Transformation2d world_to_vehicle_;
  std::vector<CameraDetection> detections_;
};
