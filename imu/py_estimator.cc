// Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

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
      bp::init<double, double, double>(
          (bp::arg("process_noise_gyro"),
           bp::arg("process_noise_bias"),
           bp::arg("measurement_noise_accel"))))
      .def("state_names", &imu::PitchEstimator::state_names)
      .def("pitch", &imu::PitchEstimator::pitch_rad)
      .def("pitch_error", &imu::PitchEstimator::pitch_error)
      .def("gyro_bias_rps", &imu::PitchEstimator::gyro_bias_rps)
      .def("covariance_diag", &imu::PitchEstimator::covariance_diag)
      .def("process_gyro", &imu::PitchEstimator::ProcessGyro)
      .def("process_accel", &imu::PitchEstimator::ProcessAccel)
      ;

  bp::class_<imu::AttitudeEstimator>(
      "AttitudeEstimator",
      bp::init<double, double, double>(
          (bp::arg("process_noise_gyro"),
           bp::arg("process_noise_bias"),
           bp::arg("measurement_noise_accel"))))
      .def("state_names", &imu::AttitudeEstimator::state_names)
      .def("pitch", &imu::AttitudeEstimator::pitch_rad)
      .def("pitch_error", &imu::AttitudeEstimator::pitch_error)
      .def("gyro_bias_rps", &imu::AttitudeEstimator::gyro_bias_rps)
      .def("covariance_diag", &imu::AttitudeEstimator::covariance_diag)
      .def("process_gyro", &imu::AttitudeEstimator::ProcessGyro,
           (bp::arg("yaw_rps"), bp::arg("pitch_rps"), bp::arg("roll_rps")))
      .def("process_accel", &imu::AttitudeEstimator::ProcessAccel)
      ;
}
