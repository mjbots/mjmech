// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#pragma once

#include <functional>

#include "ukf_filter.h"
#include "quaternion.h"

typedef double Float;

namespace imu {
Float DegToRad(Float val) {
  return val / static_cast<Float>(180.0) * static_cast<Float>(M_PI);
}

class AttitudeEstimator {
 public:
  typedef UkfFilter<Float, 7> Filter;

  AttitudeEstimator(Float process_noise_gyro,
                    Float process_noise_bias,
                    Float measurement_noise_accel)
      : initialized_(false),
        filter_(
          (Filter::State() <<
           1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0).finished(),
          Eigen::DiagonalMatrix<Float, 7, 7>(
              (Filter::State() <<
               1e-3, 1e-3, 1e-3, 1e-3,
               std::pow(DegToRad(0.001), 2),
               std::pow(DegToRad(0.001), 2),
               std::pow(DegToRad(0.001), 2)).finished()),
          Eigen::DiagonalMatrix<Float, 7, 7>(
              (Filter::State() <<
               process_noise_gyro,
               process_noise_gyro,
               process_noise_gyro,
               process_noise_gyro,
               process_noise_bias,
               process_noise_bias,
               process_noise_bias).finished())),
        measurement_noise_accel_(measurement_noise_accel) {
  }

  std::vector<std::string> state_names() const {
    return { "w", "x", "y", "z", "gx", "gy", "gz" };
  }

  Filter::Covariance covariance() const {
    return filter_.covariance();
  }

  std::vector<Float> covariance_diag() const {
    std::vector<Float> result;
    for (int i = 0; i < 7; i++) {
      result.push_back(filter_.covariance()(i, i));
    }
    return result;
  }

  Float pitch_error(Float pitch) const {
    return (attitude() *
            Quaternion<Float>::FromEuler(0., pitch, 0).
            conjugated()).euler().pitch_rad;
  }

  Float pitch_rad() const { return attitude().euler().pitch_rad; }
  Float gyro_bias_rps() const { return filter_.state()(4, 0); }

  Quaternion<Float> attitude() const {
    return Quaternion<Float>(filter_.state()(0),
                             filter_.state()(1),
                             filter_.state()(2),
                             filter_.state()(3));
  }

  Filter::State ProcessFunction(
      const Filter::State& state, Float dt_s) const {
    Filter::State result = state;

    Quaternion<Float> this_attitude = Quaternion<Float>(
        result(0), result(1), result(2), result(3)).normalized();
    Quaternion<Float> delta;
    for (const auto& x: current_gyro_) {
      Quaternion<Float> advanced = Quaternion<Float>::IntegrateRotationRate(
          x.roll_rps + result(6),
          x.pitch_rps + result(5),
          x.yaw_rps + result(4),
          dt_s);
      delta = delta * advanced;
    }

    Quaternion<Float> next_attitude = (this_attitude * delta).normalized();
    result(0) = next_attitude.w();
    result(1) = next_attitude.x();
    result(2) = next_attitude.y();
    result(3) = next_attitude.z();
    return result;
  }

  void ProcessGyro(Float yaw_rps, Float pitch_rps, Float roll_rps) {
    current_gyro_.push_back(Gyro(yaw_rps, pitch_rps, roll_rps));
  }

  static Eigen::Matrix<Float, 3, 1> OrientationToAccel(
      const Quaternion<Float>& attitude) {
    Quaternion<Float>::Vector3D gravity(0., 0., 1.);
    Quaternion<Float>::Vector3D expected =
        attitude.conjugated().Rotate(gravity);
    return expected;
  }

  static Eigen::Matrix<Float, 3, 1> MeasureAccel(
      const Filter::State& s) {
    return OrientationToAccel(
        Quaternion<Float>(s(0), s(1), s(2), s(3)).normalized());
  }

  static Quaternion<Float> AccelToOrientation(
      Float x, Float y, Float z) {
    Float roll = std::atan2(-x, z);
    Float pitch = std::atan2(y, std::sqrt(x * x + z * z));

    return Quaternion<Float>::FromEuler(roll, pitch, 0.0);
  }

  void ProcessAccel(Float x_g, Float y_g, Float z_g) {
    Float norm = std::sqrt(x_g * x_g + y_g * y_g + z_g * z_g);

    x_g /= norm;
    y_g /= norm;
    z_g /= norm;

    if (!initialized_) {
      initialized_ = true;
      Quaternion<Float> start = AccelToOrientation(x_g, y_g, z_g);
      filter_.state()(0) = start.w();
      filter_.state()(1) = start.x();
      filter_.state()(2) = start.y();
      filter_.state()(3) = start.z();
    }

    using namespace std::placeholders;
    filter_.UpdateState(
        0.01, std::bind(&AttitudeEstimator::ProcessFunction, this, _1, _2));
    filter_.UpdateMeasurement(
        MeasureAccel,
        (Eigen::Matrix<Float, 3, 1>() << x_g, y_g, z_g).finished(),
        (Eigen::DiagonalMatrix<Float, 3, 3>(
            (Eigen::Matrix<Float, 3, 1>() <<
             measurement_noise_accel_,
             measurement_noise_accel_,
             measurement_noise_accel_).finished())));
    current_gyro_.clear();
  }

 private:
  bool initialized_;
  Filter filter_;
  Float measurement_noise_accel_;

  struct Gyro {
    Float yaw_rps;
    Float pitch_rps;
    Float roll_rps;

    Gyro(Float yaw_rps, Float pitch_rps, Float roll_rps)
        : yaw_rps(yaw_rps), pitch_rps(pitch_rps), roll_rps(roll_rps) {}
  };

  std::vector<Gyro> current_gyro_;
};


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

  Float pitch_error(double pitch) {
    return pitch - pitch_rad();
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
            0., pitch_rad, 0.).conjugated().Rotate(gravity);
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
