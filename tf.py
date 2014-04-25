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
        if isinstance(x, list) and y == 0.0 and z == 0.0:
            self.x = x[0]
            self.y = x[1]
            self.z = x[2]
        else:
            self.x = x
            self.y = y
            self.z = z

    def copy(self):
        return Point3D(self.x, self.y, self.z)

    def __getitem__(self, index):
        return [self.x, self.y, self.z][index]

    def __setitem__(self, key, value):
        if key == 0:
            self.x = value
        elif key == 1:
            self.y = value
        elif key == 2:
            self.z = value
        else:
            raise IndexError()

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
        conjugated = self.rotation.conjugated()
        return Transform(
            conjugated.rotate(self.translation.scaled(-1.0)),
            conjugated)

    def copy(self):
        return Transform(self.translation.copy(), self.rotation.copy())

class FrameRelationship(object):
    '''This maintains the relationship between two frames, and allows
    a transform between the two frames to be re-calculated over time
    as it may change.

    If the parents of any frame are changed, it may invalidate
    outstanding FrameRelationship instances.
    '''
    def __init__(self, source_frame, destination_frame):
        self.source_parents_list = []
        source_parents_dict = {}
        node = source_frame
        while node is not None:
            self.source_parents_list.append(node)
            source_parents_dict[node] = True
            node = node.parent

        self.destination_parents_list = []
        node = destination_frame
        while node is not None:
            if node in source_parents_dict:
                index = self.source_parents_list.index(node)
                del self.source_parents_list[index:]
                break
            self.destination_parents_list.append(node)
            node = node.parent

    def transform(self):
        result = Transform(Point3D(0., 0., 0.), Quaternion())
        for node in self.source_parents_list:
            result = node.map_to_parent(result)

        for node in reversed(self.destination_parents_list):
            result = node.map_from_parent(result)

        return result

class Frame(object):
    '''A reference frame is a transform which defines the relationship
    between coordinates specified in this frame and a parent frame.'''
    def __init__(self, translation=None, rotation=None, parent=None):
        if translation is None:
            translation = Point3D(0., 0., 0.)
        if rotation is None:
            rotation = Quaternion()
        self.transform = Transform(translation, rotation)
        self.parent = parent

    def __repr__(self):
        return '<Frame t=%r r=%r>' % (self.transform.translation,
                                      self.transform.rotation)

    def map_to_parent(self, point_or_transform):
        '''Given a point or transform measured in this reference
        frame, map it into one in the parent's reference frame.'''
        return self.transform.apply(point_or_transform)

    def map_from_parent(self, point_or_transform):
        '''The inverse of 'map_to_parent'.'''
        return self.transform.inverse(point_or_transform)

    def relation_to_frame(self, other_frame):
        '''Return a relationship between frames that can be used
        repeatedly as the transform changes.'''
        return FrameRelationship(self, other_frame)

    def transform_to_frame(self, other_frame):
        '''Return a transform which converts coordinates specified in
        this reference frame into coordinates specified in
        'other_frame'.'''
        return self.relation_to_frame(other_frame).transform()

    def map_to_frame(self, other_frame, point_or_transform):
        '''Given a point or tranform measured in this reference frame,
        map it into an arbitrary other reference frame.'''
        return self.transform_to_frame(other_frame).apply(point_or_transform)

    def map_from_frame(self, other_frame, point_or_transform):
        '''The inverse of 'map_to_frame'.'''
        return self.transform_to_frame(other_frame).inversed().apply(
            point_or_transform)
