// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#pragma once

#include <functional>

#include "ukf_filter.h"
#include "quaternion.h"

typedef float Float;

namespace imu {
class PitchEstimator {
 public:
  typedef UkfFilter<Float, 2> PitchFilter;

  PitchEstimator(Float process_noise_gyro,
                 Float process_noise_bias,
                 Float measurement_noise_accel)
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

  std::vector<Float> covariance_diag() const {
    std::vector<Float> result;
    for (int i = 0; i < 2; i++) {
      result.push_back(pitch_filter_.covariance()(i, i));
    }
    return result;
  }

  Float pitch_rad() const { return pitch_filter_.state()(0); }
  Float gyro_bias_rps() const { return pitch_filter_.state()(1); }

  void ProcessGyro(Float gyro_rps) {
    current_gyro_.push_back(gyro_rps);
  }

  PitchFilter::State ProcessFunction(
      const PitchFilter::State& state, Float dt_s) const {
    PitchFilter::State result = state;

    Float delta = 0.0;
    for (Float x: current_gyro_) {
      delta += (x + result[1]) * dt_s;
    }

    result[0] += delta;
    return result;
  }

  static Eigen::Matrix<Float, 2, 1> OrientationToAccel(
      Float pitch_rad) {
    Quaternion<Float>::Vector3D gravity(0., 0., 1.);
    Quaternion<Float>::Vector3D expected =
        Quaternion<Float>::FromEuler(
            0., pitch_rad, 0.).Conjugated().Rotate(gravity);
    return (Eigen::Matrix<Float, 2, 1>() <<
            expected(1), expected(2)).finished();
  }

  static Eigen::Matrix<Float, 2, 1> MeasureAccel(
      const PitchFilter::State& s) {
    return OrientationToAccel(s(0));
  }

  void ProcessAccel(Float y_g, Float z_g) {
    Float norm = std::sqrt(y_g * y_g + z_g * z_g);

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
        (Eigen::Matrix<Float, 2, 1>() << y_g, z_g).finished(),
        (Eigen::Matrix<Float, 2, 2>() <<
         measurement_noise_accel_, static_cast<Float>(0.0),
         static_cast<Float>(0.0), measurement_noise_accel_).finished());

    current_gyro_.clear();
  }

 private:
  Float process_noise_gyro_;
  Float process_noise_bias_;
  Float measurement_noise_accel_;

  PitchFilter pitch_filter_;
  std::vector<Float> current_gyro_; // TODO jpieper - no malloc
};
}
