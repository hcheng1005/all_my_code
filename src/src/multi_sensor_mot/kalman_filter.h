#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

template <uint32_t XN>
class KalmanFilter {
 public:
  KalmanFilter() = default;
  KalmanFilter(const Eigen::Matrix<double, XN, 1>& state,
               const Eigen::Matrix<double, XN, XN>& covariance)
      : state_(state), covariance_(covariance) {}

  void set_state(const Eigen::Matrix<double, XN, 1>& state) { state_ = state; }

  void set_covariance(const Eigen::Matrix<double, XN, XN>& covariance) {
    covariance_ = covariance;
  }

  const Eigen::Matrix<double, XN, 1>& state() const { return state_; }

  const Eigen::Matrix<double, XN, XN>& covariance() const {
    return covariance_;
  }

  void Predict(const Eigen::Matrix<double, XN, XN>& transition_matrix,
               const Eigen::Matrix<double, XN, XN>& prediction_cov) {
    state_ = transition_matrix * state_;
    covariance_ =
        transition_matrix * covariance_ * transition_matrix.transpose() +
        prediction_cov;
  }

  template <uint32_t ZN>
  void Update(const Eigen::Matrix<double, ZN, 1>& measurement,
              const Eigen::Matrix<double, ZN, ZN>& measure_noise_cov,
              const Eigen::Matrix<double, ZN, XN>& measure_matrix) {
    const Eigen::Matrix<double, ZN, 1> innovation =
        measurement - measure_matrix * state_;
    const Eigen::Matrix<double, ZN, ZN> innovation_covariance =
        measure_matrix * covariance_ * measure_matrix.transpose() +
        measure_noise_cov;

    const Eigen::Matrix<double, ZN, ZN> innovation_covariance_inv =
        (innovation_covariance +
         Eigen::Matrix<double, ZN, ZN>::Identity() * (1e-6))
            .inverse();
    const Eigen::Matrix<double, XN, ZN> kalman_gain =
        covariance_ * measure_matrix.transpose() * innovation_covariance_inv;

    state_ += kalman_gain * innovation;
    covariance_ -= kalman_gain * measure_matrix * covariance_;
  }

 private:
  Eigen::Matrix<double, XN, 1> state_;
  Eigen::Matrix<double, XN, XN> covariance_;
};
