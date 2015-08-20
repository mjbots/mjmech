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

#include "common.h"
#include "tf.h"
#include "visitor.h"

namespace legtool {
struct JointAngles {
  struct Joint {
    int ident = 0;
    double angle_deg = 0;

    Joint() {}
    Joint(int ident, double angle_deg) : ident(ident), angle_deg(angle_deg) {}
    bool operator==(const Joint& rhs) {
      return ident == rhs.ident && angle_deg == rhs.angle_deg;
    }
  };

  std::vector<Joint> joints;

  static JointAngles Invalid() { return JointAngles(); }
  bool Valid() const { return !joints.empty(); }

  double GetLargestChangeDeg(const JointAngles& other) const {
    BOOST_ASSERT(joints.size() == other.joints.size());
    double result = 0.0;
    for (size_t i = 0; i < joints.size(); i++) {
      BOOST_ASSERT(joints[i].ident == other.joints[i].ident);
      result = std::max(
          result, std::abs(joints[i].angle_deg - other.joints[i].angle_deg));
    }
    return result;
  }
};

class IKSolver : boost::noncopyable {
 public:
  virtual ~IKSolver() {}

  /// Given a target end position in 3D coordinate space, return the
  /// required joint angles for a 3 degree of freedom leg.
  ///
  ///  +y is away from the shoulder
  ///  +x is clockwise from shoulder
  ///  +z is up
  ///
  /// If no solution is possible, return an object with NaN values.
  virtual JointAngles Solve(const Point3D&) const = 0;
};

/// Inverse kinematics solver for lizard style 3-dof legs.
class LizardIK : public IKSolver {
 public:
  struct Config {
    struct Joint {
      double min_deg = 0;
      double idle_deg = 0;
      double max_deg = 0;
      double length_mm = 0;
      double sign = 1;
      int ident = 0;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(LT_NVP(min_deg));
        a->Visit(LT_NVP(idle_deg));
        a->Visit(LT_NVP(max_deg));
        a->Visit(LT_NVP(length_mm));
        a->Visit(LT_NVP(sign));
        a->Visit(LT_NVP(ident));
      }
    };

    Joint coxa;
    Joint femur;
    Joint tibia;

    double servo_speed_dps = 360.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(LT_NVP(coxa));
      a->Visit(LT_NVP(femur));
      a->Visit(LT_NVP(tibia));
      a->Visit(LT_NVP(servo_speed_dps));
    }
  };

  LizardIK(const Config& config) : config_(config) {}

  virtual JointAngles Solve(const Point3D& point_mm) const {
    // Solve for the coxa first, as it has only a single solution.
    const double coxa_deg = (config_.coxa.sign *
                             Degrees(std::atan2(point_mm.x, point_mm.y)) +
                             config_.coxa.idle_deg);

    if (coxa_deg < config_.coxa.min_deg ||
                   coxa_deg > config_.coxa.max_deg) {
      return JointAngles::Invalid();
    }

    // x-coordinate of femur/tibia pair after rotating to 0 coxa
    const double true_x = (std::sqrt(std::pow(point_mm.x, 2) +
                                     std::pow(point_mm.y, 2)) -
                           config_.coxa.length_mm);
    const double im = std::sqrt(std::pow(point_mm.z, 2) +
                                std::pow(true_x, 2));


    // The new femur/tibia pair makes a triangle where the 3rd side is
    // the hypotenuse of the right triangle composed of z and im, lets
    // call it c.
    //
    //           --\  femur
    //           |\ --
    //           | \   --
    //           |  --    |
    //          z|  im\   | tibia
    //           |     --\|
    //           ----------
    //            true_x
    //
    // im = math.sqrt(z ** 2 + true_x ** 2)
    //
    // Then, we can use the law of cosines to find the angle opposite
    // im, which is the angle between the femur and tibia.
    //
    // im ** 2 = a ** 2 + b ** 2 + 2 * a * b * cos(C)
    //
    // Solving for C yields:
    //
    //  C = acos((im ** 2 - a ** 2 - b ** 2) / (2 * a * b))

    const double tibia_cos =
        ((std::pow(im, 2) -
          std::pow(config_.tibia.length_mm, 2) -
          std::pow(config_.femur.length_mm, 2)) /
         (2 * config_.tibia.length_mm * config_.femur.length_mm));
    if (tibia_cos < -1.0 or tibia_cos > 1.0) {
      return JointAngles::Invalid();
    }

    // For our purposes, a 0 tibia angle should equate to a right angle
    // with the femur, so subtract off 90 degrees.
    const double tibia_deg =
        (config_.tibia.sign *
         Degrees(0.5 * M_PI - std::acos(tibia_cos)) +
         config_.tibia.idle_deg);

    if (tibia_deg < config_.tibia.min_deg ||
        tibia_deg > config_.tibia.max_deg) {
      return JointAngles::Invalid();
    }

    // To solve for the femur angle, we first get the angle opposite
    // true_x, then the angle opposite the tibia.
    const double true_x_deg = Degrees(std::atan2(true_x, -point_mm.z));

    // Then the angle opposite the tibia is also found the via the law
    // of cosines.
    //
    //  tibia ** 2 = femur ** 2 + im ** 2 + 2 * femur * im * cos(femur_im)
    //
    //  femur_im = acos ( (tibia ** 2 - im ** 2 - femur ** 2) /
    //                    (2 * femur * im) )

    const double femur_im_cos =
        -(std::pow(config_.tibia.length_mm, 2) -
          std::pow(config_.femur.length_mm, 2) -
          std::pow(im, 2)) / (2 * config_.femur.length_mm * im);
    if (femur_im_cos < -1.0 || femur_im_cos > 1.0) {
      return JointAngles::Invalid();
    }

    const double femur_im_deg = Degrees(std::acos(femur_im_cos));

    const double femur_deg =
        (config_.femur.sign * ((femur_im_deg + true_x_deg) - 90.0) +
         config_.femur.idle_deg);

    if (femur_deg < config_.femur.min_deg ||
        femur_deg > config_.femur.max_deg) {
      return JointAngles::Invalid();
    }

    JointAngles result;
    result.joints.emplace_back(
        JointAngles::Joint{config_.coxa.ident, coxa_deg});
    result.joints.emplace_back(
        JointAngles::Joint{config_.femur.ident, femur_deg});
    result.joints.emplace_back(
        JointAngles::Joint{config_.tibia.ident, tibia_deg});
    return result;
  }

  const Config& config() const { return config_; }

 private:
  Config config_;
};
}
