// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <boost/python.hpp>

#include <boost/format.hpp>

#include "base/point3d.h"
#include "base/tf.h"

#include "py_legtool.h"

using namespace mjmech::base;

namespace {
const Frame* GetFrameParent(const Frame* frame) {
  return frame->parent;
}
}

void ExportTf() {
  using namespace boost::python;

  class_<Point3D>("Point3D", init<double, double, double>())
      .def(init<>())
      .def_readwrite("x", &Point3D::x)
      .def_readwrite("y", &Point3D::y)
      .def_readwrite("z", &Point3D::z)
      .def("__str_", &Point3D::str)
      .def("length", &Point3D::length)
      .def("length_squared", &Point3D::length_squared)
      .def("scaled", &Point3D::scaled)
      .def("__getitem__", &Point3D::operator[])
      .def(self + self)
      .def(self - self)
      .def(self == self)
      ;

  class_<Euler>("Euler")
      .def_readwrite("roll", &Euler::roll)
      .def_readwrite("pitch", &Euler::pitch)
      .def_readwrite("yaw", &Euler::yaw)
      ;

  class_<Quaternion>("Quaternion")
      .def_readwrite("w", &Quaternion::w)
      .def_readwrite("x", &Quaternion::x)
      .def_readwrite("y", &Quaternion::y)
      .def_readwrite("z", &Quaternion::z)
      .def("euler_rad", &Quaternion::euler_rad)
      ;

  class_<Transform>("Transform")
      .def_readwrite("translation", &Transform::translation)
      .def_readwrite("rotation", &Transform::rotation)
      ;

  class_<Frame, boost::noncopyable>("Frame")
      .def_readwrite("transform", &Frame::transform)
      .add_property("parent",
                    make_function(&GetFrameParent,
                                  return_internal_reference<1>()))
      .def("map_to_frame", &Frame::MapToFrame<Point3D>)
      .def("map_from_frame", &Frame::MapFromFrame<Point3D>)
      .def("map_to_parent", &Frame::MapToParent<Point3D>)
      .def("map_from_parent", &Frame::MapFromParent<Point3D>)
      ;
}
