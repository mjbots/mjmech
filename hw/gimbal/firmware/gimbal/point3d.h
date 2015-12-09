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

#include <assert.h>

#include "base/visitor.h"

struct Point3D {
  float x;
  float y;
  float z;

  Point3D() : x(), y(), z() {}
  Point3D(float x, float y, float z) : x(x), y(y), z(z) {}

  float operator[](size_t index) const {
    switch (index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
    }
    assert(false);
  }

  Point3D operator+(const Point3D& other) const {
    return Point3D{x + other.x, y + other.y, z + other.z};
  }

  Point3D& operator+=(const Point3D& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Point3D operator-(const Point3D& other) const {
    return Point3D{x - other.x, y - other.y, z - other.z};
  }

  bool operator==(const Point3D& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  float length() const {
    return std::sqrt(length_squared());
  }

  float length_squared() const {
    return x * x + y * y + z * z;
  }

  Point3D scaled(float value) const {
    return Point3D{x * value, y * value, z * value};
  }

  Point3D cross(const Point3D& rhs) const {
    return Point3D{
      y * rhs.z - z * rhs.y,
          z * rhs.x - x * rhs.z,
          x * rhs.y - y * rhs.x};
  }

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(x));
    a->Visit(MJ_NVP(y));
    a->Visit(MJ_NVP(z));
  }
};
