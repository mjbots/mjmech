// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#pragma once

#include <functional>

#include "ukf_filter.h"
#include "quaternion.h"

namespace imu {
class PitchEstimator {
 public:
  typedef UkfFilter<double, 2> PitchFilter;

  PitchEstimator(double process_noise_gyro,
                 double process_noise_bias,
                 double measurement_noise_accel)
      : process_noise_gyro_(process_noise_gyro),
        process_noise_bias_(process_noise_bias),
        measurement_noise_accel_(measurement_noise_accel),
        pitch_filter_(
            PitchFilter::State::Zero(),
            (PitchFilter::Covariance() <<
             std::pow(5. / 180 * M_PI, 2.0), 0.0,
             0.0, std::pow(1.0 / 180 * M_PI, 2.0)).finished(),
            (PitchFilter::Covariance() <<
             process_noise_gyro, 0.0,
             0.0, process_noise_bias).finished()) {
  }

  std::vector<std::string> state_names() const {
    return { "pitch", "gyro_bias" };
  }

  PitchFilter::Covariance covariance() const {
    return pitch_filter_.covariance();
  }

  std::vector<double> covariance_diag() const {
    std::vector<double> result;
    for (int i = 0; i < 2; i++) {
      result.push_back(pitch_filter_.covariance()(i, i));
    }
    return result;
  }

  double pitch_rad() const { return pitch_filter_.state()(0); }
  double gyro_bias_rps() const { return pitch_filter_.state()(1); }

  void ProcessGyro(double gyro_rps) {
    current_gyro_.push_back(gyro_rps);
  }

  PitchFilter::State ProcessFunction(
      const PitchFilter::State& state, double dt_s) const {
    PitchFilter::State result = state;

    double delta = 0.0;
    for (double x: current_gyro_) {
      delta += (x + result[1]) * dt_s;
    }

    result[0] += delta;
    return result;
  }

  static Eigen::Matrix<double, 2, 1> OrientationToAccel(
      double pitch_rad) {
    Eigen::Vector3d gravity(0., 0., 1.);
    Eigen::Vector3d expected = Quaternion<double>::FromEuler(
        0., pitch_rad, 0.).Conjugated().Rotate(gravity);
    return (Eigen::Matrix<double, 2, 1>() <<
            expected(1), expected(2)).finished();
  }

  static Eigen::Matrix<double, 2, 1> MeasureAccel(
      const PitchFilter::State& s) {
    return OrientationToAccel(s(0));
  }

  void ProcessAccel(double y_g, double z_g) {
    double norm = std::sqrt(y_g * y_g + z_g * z_g);

    y_g /= norm;
    z_g /= norm;

    // TODO jpieper: If this is the very first time, we should just
    // set the pitch to be the exact value as measured by the
    // accelerometers.

    using namespace std::placeholders;
    pitch_filter_.UpdateState(
        0.01, std::bind(&PitchEstimator::ProcessFunction, this, _1, _2));
    pitch_filter_.UpdateMeasurement(
        MeasureAccel,
        (Eigen::Matrix<double, 2, 1>() << y_g, z_g).finished(),
        (Eigen::Matrix<double, 2, 2>() <<
         measurement_noise_accel_, 0.0,
         0.0, measurement_noise_accel_).finished());

    current_gyro_.clear();
  }

 private:
  double process_noise_gyro_;
  double process_noise_bias_;
  double measurement_noise_accel_;

  PitchFilter pitch_filter_;
  std::vector<double> current_gyro_; // TODO jpieper - no malloc
};
}
