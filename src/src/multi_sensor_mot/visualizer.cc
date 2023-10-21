
#include "visualizer.h"

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// If some complier errors occur on the above line, please try to change it to #include <opencv2/imgproc/imgproc_c.h>

#include <chrono>
#include <random>

namespace {

const cv::Scalar kYellowColor = cv::Scalar(0, 255, 255);
const cv::Scalar kGreenColor = cv::Scalar(0, 255, 0);
const cv::Scalar kRedColor = cv::Scalar(0, 0, 255);
const cv::Scalar kBlueColor = cv::Scalar(255, 0, 0);
const cv::Scalar kBlackColor = cv::Scalar(0, 0, 0);
const cv::Scalar kWhileColor = cv::Scalar(255, 255, 255);
const cv::Scalar kGrayColor = cv::Scalar(80, 80, 80);

std::string CategoryToString(const Category& category) {
  switch (category) {
    case Category::kUnknwon:
      return "Unknown";
    case Category::kPerson:
      return "Person";
    case Category::kVehicle:
      return "Vehicle";
    default:
      return "Unknown";
  }
}

}  // namespace

Visualizer::Visualizer(const CameraFrame& camera_frame,
                       const std::vector<Track>& tracks,
                       const std::string scene_name)
    : timestamp_(camera_frame.timestamp()),
      world_to_vehicle_(camera_frame.world_to_vehicle()),
      bev_image_(cv::Mat(kImageHeight, kImageWidth, CV_8UC3, kBlackColor)),
      camera_image_(cv::imread(camera_frame.image_filename())) {
  cv::putText(camera_image_, "CAM_FRONT", cv::Point(30, 60),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 2, kGreenColor, 2, CV_AA);
  cv::putText(camera_image_, std::to_string(camera_frame.timestamp()),
              cv::Point(30, 120), cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
              kGreenColor, 2, CV_AA);
  DrawGrids();
  DrawCoordinates();
  DrawTracks(tracks);

  DrawDetectionsOnImage(camera_frame);
  DrawTrackBBoxOnImage(camera_frame, tracks);

  SaveImages(camera_frame.index(), tracks.size(), scene_name);
}

void Visualizer::DrawTracks(const std::vector<Track>& tracks) {
  for (const Track& track : tracks) {
    // if (!track.IsConfirmed()) continue;
    const std::vector<Vec2d> corners = track.GetCorners();
    std::vector<cv::Point> corners_image;

    cv::Point id_pos;
    for (int i = 0; i < corners.size(); ++i) {
      const Vec2d corner_vehicle = world_to_vehicle_.Transform(corners[i]);
      const cv::Point p = FromVehicleToImage(corner_vehicle);
      if (IsPointBeyondGridImage(p)) break;
      corners_image.push_back(p);
      if (i == 3) id_pos = p;
    }
    if (corners_image.size() < 4) continue;
    cv::polylines(bev_image_, corners_image, true, kRedColor, 1, 8, 0);
    cv::putText(bev_image_, std::to_string(track.id()),
                cv::Point(id_pos.x + 2, id_pos.y - 2), cv::FONT_HERSHEY_PLAIN,
                1, kRedColor, 1, CV_AA);

    const Vec2d center_vehicle =
        world_to_vehicle_.Transform(track.position().xy());
    const cv::Point center_image = FromVehicleToImage(center_vehicle);
    const Vec2d velocity_vehicle =
        center_vehicle + world_to_vehicle_.Rotate(track.velocity().xy());
    const cv::Point velocity_end = FromVehicleToImage(velocity_vehicle);
    cv::arrowedLine(bev_image_, center_image, velocity_end, kRedColor, 1, 8, 0,
                    0.1);
  }
}

void Visualizer::DrawCoordinates() {
  cv::putText(bev_image_, "T: " + std::to_string(timestamp_), cv::Point(20, 50),
              cv::FONT_HERSHEY_PLAIN, 2, kRedColor, 1, CV_AA);

  const cv::Point frame_x_axis_min =
      FromVehicleToImage(Vec2d(kMaxBackRange, 0));
  const cv::Point frame_x_axis_max =
      FromVehicleToImage(Vec2d(kMaxFrontRange, 0));
  const cv::Point frame_y_axis_min =
      FromVehicleToImage(Vec2d(0, kMaxRightRange));
  const cv::Point frame_y_axis_max =
      FromVehicleToImage(Vec2d(0, kMaxLeftRange));

  const cv::Point label_30m = FromVehicleToImage(Vec2d(30, 0));
  const cv::Point label_60m = FromVehicleToImage(Vec2d(60, 0));

  cv::arrowedLine(bev_image_, frame_x_axis_min, frame_x_axis_max, kRedColor, 2,
                  8, 0, 0.05);
  cv::arrowedLine(bev_image_, frame_y_axis_min, frame_y_axis_max, kBlueColor, 2,
                  8, 0, 0.05);
  cv::putText(bev_image_, "x",
              cv::Point(frame_x_axis_max.x + 25, frame_x_axis_max.y + 25),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, kRedColor, 1, CV_AA);
  cv::putText(bev_image_, "y",
              cv::Point(frame_y_axis_max.x, frame_y_axis_max.y - 15),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, kBlueColor, 1, CV_AA);

  cv::putText(bev_image_, "30m", cv::Point(label_30m.x + 5, label_30m.y),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, kRedColor, 1, CV_AA);
  cv::putText(bev_image_, "60m", cv::Point(label_60m.x + 5, label_60m.y),
              cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, kRedColor, 1, CV_AA);
}

void Visualizer::DrawGrids() {
  const double base_size = 5.0;  // meters
  for (double row = kMaxBackRange; row <= kMaxFrontRange; row += base_size) {
    const int row_p = static_cast<int>((row - kMaxBackRange) / kGridSize);
    const cv::Point p1 = cv::Point(0, row_p);
    const cv::Point p2 = cv::Point(kImageWidth, row_p);
    cv::line(bev_image_, p1, p2, kGrayColor, 1);
  }
  for (double col = kMaxRightRange; col <= kMaxLeftRange; col += base_size) {
    const int col_p = static_cast<int>((col - kMaxRightRange) / kGridSize);
    const cv::Point p1 = cv::Point(col_p, 0);
    const cv::Point p2 = cv::Point(col_p, kImageHeight);
    cv::line(bev_image_, p1, p2, kGrayColor, 1);
  }
}

void Visualizer::SaveImages(const int frame_index, const int tracks_num,
                            const std::string scene_name) {
  const int final_rows = std::max(bev_image_.rows, camera_image_.rows);
  const int final_cols = bev_image_.cols + camera_image_.cols;

  final_image_ = cv::Mat(final_rows, final_cols, CV_8UC3, kBlackColor);

  const int row_offset =
      std::max(static_cast<int>((bev_image_.rows - camera_image_.rows) / 2), 0);

  const cv::Rect camera_region =
      cv::Rect(0, row_offset, camera_image_.cols, camera_image_.rows);
  const cv::Rect bev_region =
      cv::Rect(camera_image_.cols, 0, bev_image_.cols, bev_image_.rows);

  camera_image_.copyTo(final_image_(camera_region));
  bev_image_.copyTo(final_image_(bev_region));

  const std::string filename = "../visualization/" + scene_name + "_frame_" +
                               std::to_string(frame_index) + "_tracks_num_" +
                               std::to_string(tracks_num) + "_" +
                               std::to_string(timestamp_) + ".png";
  cv::imwrite(filename, final_image_);

  std::cout << "Save image as " << filename << std::endl;

  cv::imshow("MOT", final_image_);
  cv::waitKey(50);
}

void Visualizer::DrawTrackBBoxOnImage(const CameraFrame& camera_frame,
                                      const std::vector<Track>& tracks) {
  for (const Track& track : tracks) {
    if (track.last_camera_update_timestamp() < camera_frame.timestamp() - 1e-6)
      continue;

    AABox bbox;
    if (track.GetProjectionOnImage(world_to_vehicle_, &bbox)) {
      const cv::Rect rect = cv::Rect(bbox.top_left_x(), bbox.top_left_y(),
                                     bbox.width(), bbox.height());
      cv::rectangle(camera_image_, rect, kRedColor, 2, 1, 0);
      cv::putText(
          camera_image_,
          std::to_string(track.id()) + "," + CategoryToString(track.category()),
          cv::Point(bbox.top_left_x(), bbox.top_left_y() + bbox.height() - 5),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, kRedColor, 1, CV_AA);
    }
  }
}

void Visualizer::DrawDetectionsOnImage(const CameraFrame& camera_frame) {
  for (const CameraDetection& detection : camera_frame.detections()) {
    const AABox& bbox = detection.bbox();
    const cv::Rect rect = cv::Rect(bbox.top_left_x(), bbox.top_left_y(),
                                   bbox.width(), bbox.height());
    cv::rectangle(camera_image_, rect, kGreenColor, 2, 1, 0);
    cv::putText(camera_image_,
                std::to_string(detection.id()) + "," +
                    CategoryToString(detection.category()),
                cv::Point(bbox.top_left_x(), bbox.top_left_y() - 5),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, kGreenColor, 1, CV_AA);
  }
}