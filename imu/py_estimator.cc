// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "estimator.h"

#include <boost/python.hpp>
#include <boost/python/args.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_estimator) {
  bp::class_<std::vector<double> >("doublevec")
      .def(bp::vector_indexing_suite<std::vector<double> >())
      ;
  bp::class_<std::vector<float> >("doublefloat")
      .def(bp::vector_indexing_suite<std::vector<float> >())
      ;
  bp::class_<std::vector<std::string> >("stringvec")
      .def(bp::vector_indexing_suite<std::vector<std::string> >())
      ;

  bp::class_<imu::PitchEstimator>(
      "PitchEstimator",
      bp::init<double, double, double, double, double>(
          (bp::arg("process_noise_gyro"),
           bp::arg("process_noise_bias"),
           bp::arg("measurement_noise_accel"),
           bp::arg("initial_noise_attitude"),
           bp::arg("initial_noise_bias"))))
      .def("state_names", &imu::PitchEstimator::state_names)
      .def("set_initial_gyro_bias", &imu::PitchEstimator::SetInitialGyroBias,
           (bp::arg("yaw_rps"), bp::arg("pitch_rps"), bp::arg("roll_rps")))
      .def("yaw", &imu::PitchEstimator::yaw_rad)
      .def("pitch", &imu::PitchEstimator::pitch_rad)
      .def("roll", &imu::PitchEstimator::roll_rad)
      .def("yaw_rps", &imu::PitchEstimator::yaw_rps)
      .def("pitch_rps", &imu::PitchEstimator::pitch_rps)
      .def("roll_rps", &imu::PitchEstimator::roll_rps)
      .def("pitch_error", &imu::PitchEstimator::pitch_error)
      .def("gyro_bias_rps", &imu::PitchEstimator::gyro_bias_rps)
      .def("covariance_diag", &imu::PitchEstimator::covariance_diag)
      .def("process_measurement", &imu::PitchEstimator::ProcessMeasurement,
           (bp::arg("yaw_rps"), bp::arg("pitch_rps"), bp::arg("roll_rps"),
            bp::arg("x_g"), bp::arg("y_g"), bp::arg("z_g")))
      ;

  bp::class_<imu::AttitudeEstimator>(
      "AttitudeEstimator",
      bp::init<double, double, double, double, double>(
          (bp::arg("process_noise_gyro"),
           bp::arg("process_noise_bias"),
           bp::arg("measurement_noise_accel"),
           bp::arg("initial_noise_attitude"),
           bp::arg("initial_noise_bias"))))
      .def("state_names", &imu::AttitudeEstimator::state_names)
      .def("set_initial_gyro_bias",
           &imu::AttitudeEstimator::SetInitialGyroBias,
           (bp::arg("yaw_rps"), bp::arg("pitch_rps"), bp::arg("roll_rps")))
      .def("yaw", &imu::AttitudeEstimator::yaw_rad)
      .def("pitch", &imu::AttitudeEstimator::pitch_rad)
      .def("roll", &imu::AttitudeEstimator::roll_rad)
      .def("yaw_rps", &imu::AttitudeEstimator::yaw_rps)
      .def("pitch_rps", &imu::AttitudeEstimator::pitch_rps)
      .def("roll_rps", &imu::AttitudeEstimator::roll_rps)
      .def("pitch_error", &imu::AttitudeEstimator::pitch_error)
      .def("gyro_bias_rps", &imu::AttitudeEstimator::gyro_bias_rps)
      .def("covariance_diag", &imu::AttitudeEstimator::covariance_diag)
      .def("process_measurement", &imu::AttitudeEstimator::ProcessMeasurement,
           (bp::arg("yaw_rps"), bp::arg("pitch_rps"), bp::arg("roll_rps"),
            bp::arg("x_g"), bp::arg("y_g"), bp::arg("z_g")))
      ;
}
