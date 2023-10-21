#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

class Vec2d {
 public:
  Vec2d() : x_(0.0), y_(0.0) {}
  Vec2d(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }
  double y() const { return y_; }

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_); }
  Vec2d operator+(const Vec2d& b) const {
    return Vec2d(x_ + b.x(), y_ + b.y());
  }
  Vec2d operator-(const Vec2d& b) const {
    return Vec2d(x_ - b.x(), y_ - b.y());
  }
  Vec2d operator*(const double k) const { return Vec2d(k * x_, k * y_); }

 private:
  double x_;
  double y_;
};

class Vec3d {
 public:
  Vec3d() : x_(0.0), y_(0.0), z_(0.0) {}
  Vec3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  Vec3d(const Eigen::Vector3d& p) : x_(p[0]), y_(p[1]), z_(p[2]) {}
  Vec3d(const Vec2d& xy, const double z) : x_(xy.x()), y_(xy.y()), z_(z) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }

  Vec2d xy() const { return Vec2d(x_, y_); }

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_); }
  Vec3d operator+(const Vec3d& b) const {
    return Vec3d(x_ + b.x(), y_ + b.y(), z_ + b.z());
  }
  Vec3d operator-(const Vec3d& b) const {
    return Vec3d(x_ - b.x(), y_ - b.y(), z_ - b.z());
  }
  Vec3d operator*(const double k) const {
    return Vec3d(k * x_, k * y_, k * z_);
  }

 private:
  double x_;
  double y_;
  double z_;
};

class Transformation2d {
 public:
  Transformation2d() = default;
  ~Transformation2d() = default;

  Transformation2d(const Vec2d translation, const double theta)
      : translation_(translation), theta_(theta) {}

  Vec2d translation() const { return translation_; }
  double theta() const { return theta_; }

  Vec2d Rotate(const Vec2d point) const {
    return Vec2d(point.x() * std::cos(theta_) + point.y() * std::sin(theta_),
                 point.y() * std::cos(theta_) - point.x() * std::sin(theta_));
  }

  Vec2d Transform(const Vec2d point) const {
    const Vec2d p = point - translation_;
    return Rotate(p);
  }

 private:
  Vec2d translation_;
  double theta_;
};

class Transformation3d {
 public:
  Transformation3d() = default;
  ~Transformation3d() = default;

  Transformation3d(const Vec3d translation, const double q_w, const double q_x,
                   const double q_y, const double q_z)
      : translation_(translation),
        quaternion_(Eigen::Quaterniond(q_w, q_x, q_y, q_z)) {}

  Vec3d translation() const { return translation_; }
  Eigen::Quaterniond quaternion() const { return quaternion_; }

  Vec3d Transform(const Vec3d point) const {
    const Vec3d p = point - translation_;
    Eigen::Quaterniond pp;
    pp.w() = 0;
    pp.vec() = Eigen::Vector3d(p.x(), p.y(), p.z());
    Eigen::Quaterniond rotated_pp = quaternion_.inverse() * pp * quaternion_;
    Eigen::Vector3d rotated_p = rotated_pp.vec();
    return Vec3d(rotated_p);
  }

 private:
  Vec3d translation_;
  Eigen::Quaterniond quaternion_;
};

class CameraProjection {
 public:
  static constexpr int kImageWidthPixel = 1600;
  static constexpr int kImageHeightPixel = 900;
  static constexpr int kImageMarginPixel = 50;

  CameraProjection() = default;
  ~CameraProjection() = default;

  CameraProjection(Vec3d translation, double rotation[4], double intrinsic[4])
      : vehicle_to_camera_(Transformation3d(
            translation, rotation[0], rotation[1], rotation[2], rotation[3])),
        focal_length_x_(intrinsic[0]),
        focal_length_y_(intrinsic[2]),
        origin_shift_x_(intrinsic[1]),
        origin_shift_y_(intrinsic[3]) {}

  Vec3d VehileToCamera(const Vec3d& point) const {
    return vehicle_to_camera_.Transform(point);
  }

  bool VehicleToImage(const Vec3d& point, int* u, int* v) const {
    const Vec3d point_in_camera = vehicle_to_camera_.Transform(point);
    if (point_in_camera.z() < 0) return false;
    const double normalized_x = point_in_camera.x() / point_in_camera.z();
    const double normalized_y = point_in_camera.y() / point_in_camera.z();
    *u = normalized_x * focal_length_x_ + origin_shift_x_;
    *v = normalized_y * focal_length_y_ + origin_shift_y_;
    if (*u < -kImageMarginPixel || *u >= kImageWidthPixel + kImageMarginPixel ||
        *v < -kImageMarginPixel || *v > kImageHeightPixel + kImageMarginPixel) {
      return false;
    }
    return true;
  }

  double focal_length_x() const { return focal_length_x_; }
  double focal_length_y() const { return focal_length_y_; }
  double origin_shift_x() const { return origin_shift_x_; }
  double origin_shift_y() const { return origin_shift_y_; }

 private:
  Transformation3d vehicle_to_camera_;
  double focal_length_x_;
  double focal_length_y_;
  double origin_shift_x_;
  double origin_shift_y_;
};
