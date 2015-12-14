// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cmath>

#include <Eigen/Core>

#include "euler.h"
#include "point3d.h"

class Quaternion;
inline Quaternion operator*(const Quaternion& lhs,
                            const Quaternion& rhs);

class Quaternion {
 public:
  Quaternion(float w, float x, float y, float z)
      : w_(w), x_(x), y_(y), z_(z) {}

  Quaternion()
      : w_(1.0f), x_(0.f), y_(0.f), z_(0.f) {}

  Point3D Rotate(const Point3D& vector3d) const {
    Quaternion p(0.0f,
                 vector3d[0],
                 vector3d[1],
                 vector3d[2]);
    Quaternion q = *this * p * conjugated();
    return Point3D(q.x(), q.y(), q.z());
  }

  Quaternion conjugated() const {
    return Quaternion(w_, -x_, -y_, -z_);
  }

  Quaternion normalized() const {
    float norm = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
    return Quaternion(w_ / norm,
                      x_ / norm,
                      y_ / norm,
                      z_ / norm);
  }

  Eigen::Matrix<float, 3, 3> matrix() const {
    Eigen::Matrix<float, 3, 3> r;

    r(0, 0) = w_ * w_ + x_ * x_ - y_ * y_ - z_ * z_;
    r(0, 1) = 2 * (x_ * y_ - w_ * z_);
    r(0, 2) = 2 * (w_ * y_ + x_ * z_);
    r(1, 0) = 2 * (x_ * y_ + w_ * z_);
    r(1, 1) = w_ * w_ - x_ * x_ + y_ * y_ - z_ * z_;
    r(1, 2) = 2 * (y_ * z_ - w_ * x_);
    r(2, 0) = 2 * (x_ * z_ - w_ * y_);
    r(2, 1) = 2 * (w_ * x_ + y_ * z_);
    r(2, 2) = w_ * w_ - x_ * x_ - y_ * y_ + z_ * z_;

    return r;
  }

  /// Euler angles are in roll, pitch, then yaw.
  ///  +roll -> right side down
  ///  +pitch -> forward edge up
  ///  +yaw -> clockwise looking down
  Euler euler_rad() const {
    float sp = 2.0f * (w_ * x_ + y_ * z_);
    Euler result_rad;
    if (std::abs(sp - 1.0f) < 1e-8f) { // north pole
      result_rad.pitch = M_PI_2;
      result_rad.roll = 0.0f;
      result_rad.yaw = -std::atan2((w_ * y_ + x_ * z_),
                                   -(y_ * z_ - w_ * x_));
    } else if (std::abs(sp + 1.0f) < 1e-8f) { // south pole
      result_rad.pitch = -M_PI_2;
      result_rad.roll = 0.0f;
      result_rad.yaw = std::atan2((w_ * y_ + x_ * z_),
                                  (y_ * z_ - w_ * x_));
    } else {
      result_rad.pitch = std::asin(sp);
      result_rad.roll = -std::atan2(2.0f * (x_ * z_ - w_ * y_),
                                    1.0f - 2.0f * x_ * x_ - 2.0f * y_ * y_);
      result_rad.yaw = std::atan2(2.0f * (x_ * y_ - w_ * z_),
                                  1.0f - 2.0f * x_ * x_ - 2.0f * z_ * z_);
    }

    return result_rad;
  }

  static Quaternion FromEuler(
      float roll_rad, float pitch_rad, float yaw_rad) {
    // Quaternions multiply in opposite order, and we want to get into
    // roll, pitch, then yaw as standard.
    return (Quaternion::FromAxisAngle(yaw_rad, 0.0f, 0.0f, -1.0f) *
            Quaternion::FromAxisAngle(pitch_rad, 1.0f, 0.0f, 0.0f) *
            Quaternion::FromAxisAngle(roll_rad, 0.0f, 1.0f, 0.0f));
  }

  static Quaternion FromEuler(const Euler& euler_rad) {
    return FromEuler(euler_rad.roll, euler_rad.pitch, euler_rad.yaw);
  }

  static Quaternion FromAxisAngle(
      float angle_rad, float x, float y, float z) {
    float c = std::cos(angle_rad / 2.0f);
    float s = std::sin(angle_rad / 2.0f);

    return Quaternion(c, x * s, y * s, z * s);
  }

  static Quaternion IntegrateRotationRate(
      float roll_rate_rps, float pitch_rate_rps, float yaw_rate_rps,
      float dt_s) {
    return Quaternion(1.0f,
                      0.5f * pitch_rate_rps * dt_s,
                      0.5f * roll_rate_rps * dt_s,
                      -0.5f * yaw_rate_rps * dt_s).normalized();
  }

  static Quaternion IntegrateRotationRate(
      const Point3D& rate_rps, float dt_s) {
    return Quaternion(1.0f,
                      0.5f * rate_rps.x * dt_s,
                      0.5f * rate_rps.y * dt_s,
                      0.5f * rate_rps.z * dt_s).normalized();
  }

  float w() const { return w_; }
  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(mjmech::base::MakeNameValuePair(&w_, "w"));
    a->Visit(mjmech::base::MakeNameValuePair(&x_, "x"));
    a->Visit(mjmech::base::MakeNameValuePair(&y_, "y"));
    a->Visit(mjmech::base::MakeNameValuePair(&z_, "z"));
  }

 private:
  float w_;
  float x_;
  float y_;
  float z_;
};

inline Quaternion operator*(const Quaternion& lhs,
                            const Quaternion& rhs) {
  float a = lhs.w();
  float b = lhs.x();
  float c = lhs.y();
  float d = lhs.z();

  float e = rhs.w();
  float f = rhs.x();
  float g = rhs.y();
  float h = rhs.z();

  return Quaternion(a * e - b * f - c * g - d * h,
                    b * e + a * f + c * h - d * g,
                    a * g - b * h + c * e + d * f,
                    a * h + b * g - c * f + d * e);
}
