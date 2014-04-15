# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import math
import tf

def check_vectors_close(v1, v2):
    assert abs(v1.x - v2.x) < 0.001
    assert abs(v1.y - v2.y) < 0.001
    assert abs(v1.z - v2.z) < 0.001

def check_mapping(f1, f1p, f2p):
    check_vectors_close(f1.map_to_parent(tf.Point3D(*f1p)), tf.Point3D(*f2p))
    check_vectors_close(f1.map_from_parent(tf.Point3D(*f2p)), tf.Point3D(*f1p))

def check_frames(f1, f1p, f2, f2p):
    check_vectors_close(f1.map_to_frame(f2, tf.Point3D(*f1p)), tf.Point3D(*f2p))
    check_vectors_close(f2.map_from_frame(f1, tf.Point3D(*f1p)),
                        tf.Point3D(*f2p))
    check_vectors_close(f2.map_to_frame(f1, tf.Point3D(*f2p)), tf.Point3D(*f1p))
    check_vectors_close(f1.map_from_frame(f2, tf.Point3D(*f2p)),
                        tf.Point3D(*f1p))

def test_simple_transforms():
    frame = tf.Frame(tf.Point3D(10., 0., 0.),
                     tf.Quaternion.from_euler(0, 0, 0))

    check_mapping(frame, (0., 0., 0.), (10., 0., 0.))
    check_mapping(frame, (1., 0., 0.), (11., 0., 0.))
    check_mapping(frame, (1., 2., 0.), (11., 2., 0.))
    check_mapping(frame, (1., 2., 3.), (11., 2., 3.))

    frame = tf.Frame(tf.Point3D(0., 10., 0.),
                     tf.Quaternion.from_euler(0., 0., math.radians(90.)))
    check_mapping(frame, (0., 0., 0.), (0., 10., 0.))
    check_mapping(frame, (1., 0., 0.), (0., 9., 0.))
    check_mapping(frame, (-1., 0., -2.), (0., 11., -2.))

    frame = tf.Frame(tf.Point3D(0., 0., 3.),
                     tf.Quaternion.from_euler(math.radians(-90.), 0., 0.))
    check_mapping(frame, (0., 0., 0.), (0., 0., 3.))
    check_mapping(frame, (0., 0., 1.), (-1., 0., 3.))

def test_frame_chains():
    root = tf.Frame(tf.Point3D(0., 0., 0.), tf.Quaternion())

    child1 = tf.Frame(tf.Point3D(10., 2., 0.), tf.Quaternion(), root)
    child2 = tf.Frame(tf.Point3D(-3., -5., 0.), tf.Quaternion(), root)

    check_mapping(child1, (0., 0., 0.), (10., 2., 0.))

    check_frames(child1, (0., 0., 0.), root, (10., 2., 0.))
    check_frames(child2, (0., 0., 0.), root, (-3., -5., 0.))
    check_frames(child1, (0., 0., 0.), child2, (13., 7., 0.))

    subchild1 = tf.Frame(tf.Point3D(1., 2., 0.), tf.Quaternion(), child1)
    check_mapping(subchild1, (0., 0., 0.), (1., 2., 0.))
    check_frames(subchild1, (0., 0., 0.), child1, (1., 2., 0.))
    check_frames(subchild1, (0., 0., 0.), root, (11., 4., 0.))
    check_frames(subchild1, (0., 0., 0.), child2, (14., 9., 0.))
