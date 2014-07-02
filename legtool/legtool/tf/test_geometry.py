# Copyright 2014 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from . import geometry

from . import tf

_SQUARE = [tf.Point3D(0., 0., 0.),
           tf.Point3D(10., 0., 0.),
           tf.Point3D(10., 10., 0.),
           tf.Point3D(0., 10., 0.)]

_TRIANGLE = [tf.Point3D(-10., 0., 0.),
             tf.Point3D(10., 0., 0.),
             tf.Point3D(0., 10., 0.)]

def check_vectors_close(v1, v2):
    assert abs(v1.x - v2.x) < 0.001
    assert abs(v1.y - v2.y) < 0.001
    assert abs(v1.z - v2.z) < 0.001

def test_area():
    result = geometry.signed_poly_area(_SQUARE)
    assert abs(result - 100.0) < 0.1

    result = geometry.signed_poly_area(list(reversed(_SQUARE)))
    assert abs(result + 100.0) < 0.1

    result = geometry.signed_poly_area(_TRIANGLE)
    assert abs(result - 100.0) < 0.1

def test_centroid():
    result = geometry.poly_centroid(_SQUARE)
    check_vectors_close(result, tf.Point3D(5., 5., 0.))

    result = geometry.poly_centroid(list(reversed(_SQUARE)))
    check_vectors_close(result, tf.Point3D(5., 5., 0.))

    result = geometry.poly_centroid(_TRIANGLE)
    check_vectors_close(result, tf.Point3D(0., 3.333, 0.))

def test_point_in_poly():
    point_in_poly = geometry.point_in_poly

    assert point_in_poly(tf.Point3D(5., 5., 0.), _SQUARE) == True
    assert point_in_poly(tf.Point3D(15., 5., 0.), _SQUARE) == False
    assert point_in_poly(tf.Point3D(-5., 5., 0.), _SQUARE) == False
    assert point_in_poly(tf.Point3D(-5., 5., 0.), _SQUARE) == False

    assert point_in_poly(tf.Point3D(0., 1., 0.), _TRIANGLE) == True
    assert point_in_poly(tf.Point3D(0., -1., 0.), _TRIANGLE) == False
    assert point_in_poly(tf.Point3D(0., 9., 0.), _TRIANGLE) == True
    assert point_in_poly(tf.Point3D(0., 11., 0.), _TRIANGLE) == False
    assert point_in_poly(tf.Point3D(3., 9., 0.), _TRIANGLE) == False

def test_distance_to_segment():
    dut = geometry.distance_to_segment

    def test(qx, qy, x1, y1, x2, y2, expected):
        result = dut(tf.Point3D(qx, qy, 0.),
                     tf.Point3D(x1, y1, 0.),
                     tf.Point3D(x2, y2, 0.))
        assert abs(result - expected) < 0.01

    test(0., 0., 0., 0., 0., 0., 0.)
    test(0., 0., 0., 0., 3., 0., 0.)
    test(1., 0., 0., 0., 3., 0., 0.)
    test(1., 1., 0., 0., 3., 0., 1.)
    test(1., -1., 0., 0., 3., 0., 1.)
    test(1., -1., 3., 0., 0., 0., 1.)
    test(5., 0., 3., 0., 0., 0., 2.)

    test(0., 0., 2., 4., 2., -3., 2.)
    test(1., 0., 2., 4., 2., -3., 1.)
    test(5., 0., 2., 4., 2., -3., 3.)
    test(5., 8., 2., 4., 2., -3., 5.)

def test_distance_to_poly():
    dut = geometry.distance_to_poly

    def test(qx, qy, poly, expected):
        result = dut(tf.Point3D(qx, qy, 0.), poly)
        assert abs(result - expected) < 0.01

    test(0., 0., _SQUARE, 0.)
    test(-1., 0., _SQUARE, 1.)
    test(4., 2., _SQUARE, 2.)
