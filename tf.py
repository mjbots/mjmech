# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

'''Transformation and reference frame module.

This file contains routines for managing a tree of 6dof reference
frames, and converting between them.'''

import math

from quaternion import Quaternion

class Point3D(object):
    '''A 3 dimensional point or vector.

    The assumed coordinate system for local coordinates is:
      +x - right
      +y - forward
      +z - up
    '''
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '<Point3D x/y/z %r/%r/%r>' % (self.x, self.y, self.z)

    def __add__(self, other):
        return Point3D(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z)

    def __sub__(self, other):
        return Point3D(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z)

    def __mul__(self, other):
        return Point3D(
            self.x * other.x,
            self.y * other.y,
            self.z * other.z)

    def __eq__(self, other):
        if isinstance(other, Point3D):
            return self.x == other.x and self.y == other.y and self.z == other.z
        raise NotImplementedError()

    def __ne__(self, other):
        return not (self == other)

    def length(self):
        value = self * self
        return math.sqrt(value.x + value.y + value.z)

    def scaled(self, value):
        return Point3D(self.x * value,
                       self.y * value,
                       self.z * value)

class Transform(object):
    '''A Transform represents a mapping from one 6dof reference frame
    to another. It is constructed from a translation and rotation.
    When applying the transform, to points, the rotation is applied
    first, then the translation.

    Transforms may also be combined using the same methods.
    '''
    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

    def apply(self, other):
        '''Apply the current transform to either a point or another
        transform.'''
        if isinstance(other, Transform):
            return Transform(self.translation +
                             self.rotation.rotate(other.translation),
                             self.rotation * other.rotation)

        # Assume it is something like a point.
        return self.rotation.rotate(other) + self.translation

    def inverse(self, other):
        '''Perform the inverse operation of apply.'''
        return self.inversed().apply(other)

    def inversed(self):
        '''Return a transform which performs the inverse operation.'''
        raise NotImplementedError()

class Frame(object):
    def __init__(self, translation, rotation, parent=None):
        self.transform = Transform(translation, rotation)
        self.parent = parent

    def map_to_parent(self, point_or_transform):
        return self.transform.apply(point_or_transform)

    def map_from_parent(self, point_or_transform):
        return self.transform.inverse(point_or_transform)

    def transform_to_frame(self, other_frame):
        # TODO jpieper: Walk up to a common ancestor, then back down
        # again.
        raise NotImplementedError()

    def map_to_frame(self, other_frame, point_or_transform):
        return self.transform_to_frame(other_frame).apply(point_or_transform)

    def map_from_frame(self, other_frame, point_or_transform):
        return self.transform_to_frame(other_frame).inversed().apply(
            point_or_transform)
