// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

#pragma once

#include <cmath>

#include <boost/assert.hpp>

#include <Eigen/Core>

namespace imu {
template <typename _Scalar>
class Quaternion {
 public:
  Quaternion(_Scalar w, _Scalar x, _Scalar y, _Scalar z)
      : w_(w), x_(x), y_(y), z_(z) {}

  Quaternion()
      : w_(1.0), x_(0.), y_(0.), z_(0.) {}

  typedef Eigen::Matrix<_Scalar, 3, 1> Vector3D;

  Vector3D Rotate(const Vector3D& vector3d) const {
    Quaternion p(0.0,
                 vector3d[0],
                 vector3d[1],
                 vector3d[2]);
    Quaternion q = *this * p * conjugated();
    return Vector3D(q.x(), q.y(), q.z());
  }

  Quaternion conjugated() const {
    return Quaternion(w_, -x_, -y_, -z_);
  }

  Quaternion normalized() const {
    _Scalar norm = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
    return Quaternion(w_ / norm,
                      x_ / norm,
                      y_ / norm,
                      z_ / norm);
  }

  Eigen::Matrix<_Scalar, 3, 3> matrix() const {
    Eigen::Matrix<_Scalar, 3, 3> r;

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
  struct Euler {
    _Scalar roll_rad;
    _Scalar pitch_rad;
    _Scalar yaw_rad;

    Euler() : roll_rad(0.), pitch_rad(0.), yaw_rad(0.) {}
  };

  Euler euler() const {
    _Scalar sp = 2 * (w_ * x_ + y_ * z_);
    Euler result;
    if (std::abs(sp - 1.0) < 1e-8) { // north pole
      result.pitch_rad = M_PI_2;
      result.roll_rad = 0;
      result.yaw_rad = -std::atan2((w_ * y_ + x_ * z_),
                                   -(y_ * z_ - w_ * x_));
    } else if (std::abs(sp + 1.0) < 1e-8) { // south pole
      result.pitch_rad = -M_PI_2;
      result.roll_rad = 0;
      result.yaw_rad = std::atan2((w_ * y_ + x_ * z_),
                                  (y_ * z_ - w_ * x_));
    } else {
      result.pitch_rad = std::asin(sp);
      result.roll_rad = -std::atan2(2 * (x_ * z_ - w_ * y_),
                                    1.0 - 2 * x_ * x_ - 2 * y_ * y_);
      result.yaw_rad = std::atan2(2 * (x_ * y_ - w_ * z_),
                                  1 - 2 * x_ * x_ - 2 * z_ * z_);
    }

    return result;
  }

  static Quaternion FromEuler(
      _Scalar roll_rad, _Scalar pitch_rad, _Scalar yaw_rad) {
    // Quaternions multiply in opposite order, and we want to get into
    // roll, pitch, then yaw as standard.
    return (Quaternion::FromAxisAngle(yaw_rad, 0, 0, -1) *
            Quaternion::FromAxisAngle(pitch_rad, 1, 0, 0) *
            Quaternion::FromAxisAngle(roll_rad, 0, 1, 0));
  }

  static Quaternion FromAxisAngle(
      _Scalar angle_rad, _Scalar x, _Scalar y, _Scalar z) {
    _Scalar c = std::cos(angle_rad / 2.0);
    _Scalar s = std::sin(angle_rad / 2.0);

    return Quaternion(c, x * s, y * s, z * s);
  }

  static Quaternion IntegrateRotationRate(
      _Scalar roll_rate_rps, _Scalar pitch_rate_rps, _Scalar yaw_rate_rps,
      _Scalar dt_s) {
    // This simple technique will yield terrible results if the total
    // delta is too large.
    const _Scalar kMaxIntegrationAngle = 0.5;

    BOOST_ASSERT(roll_rate_rps * dt_s < kMaxIntegrationAngle);
    BOOST_ASSERT(pitch_rate_rps * dt_s < kMaxIntegrationAngle);
    BOOST_ASSERT(yaw_rate_rps * dt_s < kMaxIntegrationAngle);

    return Quaternion(1.0,
                      0.5 * pitch_rate_rps * dt_s,
                      0.5 * roll_rate_rps * dt_s,
                      -0.5 * yaw_rate_rps * dt_s).normalized();
  }

  _Scalar w() const { return w_; }
  _Scalar x() const { return x_; }
  _Scalar y() const { return y_; }
  _Scalar z() const { return z_; }

 private:
  _Scalar w_;
  _Scalar x_;
  _Scalar y_;
  _Scalar z_;
};

template <typename _Scalar>
Quaternion<_Scalar> operator*(const Quaternion<_Scalar>& lhs,
                              const Quaternion<_Scalar>& rhs) {
  _Scalar a = lhs.w();
  _Scalar b = lhs.x();
  _Scalar c = lhs.y();
  _Scalar d = lhs.z();

  _Scalar e = rhs.w();
  _Scalar f = rhs.x();
  _Scalar g = rhs.y();
  _Scalar h = rhs.z();

  return Quaternion<_Scalar>(a * e - b * f - c * g - d * h,
                             b * e + a * f + c * h - d * g,
                             a * g - b * h + c * e + d * f,
                             a * h + b * g - c * f + d * e);
}
}
