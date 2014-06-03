# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import tf

def signed_poly_area(poly):
    '''Given a list of points which describe a polygon in 2D, return
    the area.  A clockwise wound polygon will have a negative area,
    which a counterclockwise wound one will have a positive area.'''

    # Our polygon must be an open one.
    assert poly[0] != poly[-1]

    N = len(poly)
    total = 0.0
    for i in range(N):
        total += (poly[i].x * poly[(i + 1) % N].y -
                  poly[(i + 1) % N].x * poly[i].y)
    return total / 2.0

def poly_centroid(poly):
    '''Given a list of points which describe a polygon in 2D, return
    the centroid of that polygon.'''

    # Our polygon must be an open one.
    assert poly[0] != poly[-1]

    if len(poly) == 2:
        # This "polygon" is actually a line.  Just return the center.
        return (poly[0] + poly[1]).scaled(0.5)

    area = signed_poly_area(poly)

    sumx = 0.0
    sumy = 0.0
    N = len(poly)
    for i in range(N):
        sumx += ((poly[i].x + poly[(i + 1) % N].x) *
                 (poly[i].x * poly[(i + 1) % N].y -
                  poly[(i + 1) % N].x * poly[i].y))
        sumy += ((poly[i].y + poly[(i + 1) % N].y) *
                 (poly[i].x * poly[(i + 1) % N].y -
                  poly[(i + 1) % N].x * poly[i].y))

    return tf.Point3D(sumx / (6 * area), sumy / (6 * area), 0)

def point_in_poly(point, poly):
    '''Return True if the given point is inside the simple polygon.
    The result if the query lies directly on a segment of the polygon
    is True.'''

    # http://www.ecse.rpi.edu/~wrf/Research/Short_Notes/pnpoly.html

    N = len(poly)
    i = 0
    j = N - 1
    c = False

    while i < N:
        if ((poly[i].y > point.y) != (poly[j].y > point.y) and
            (point.x < ((poly[j].x - poly[i].x) *
                        (point.y - poly[i].y) /
                        (poly[j].y - poly[i].y) + poly[i].x))):
            c = not c
        j = i
        i += 1
    return c

def dot(v1, v2):
    '''Return the dot product of two vectors.'''
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z

def distance_to_segment(p, v, w):
    '''Return the closest distance between the query point p, and the
    segment defined by v and w.'''
    l2 = (v - w).length_squared()
    if l2 == 0.0:
        return (p - v).length()

    t = dot(p - v, w - v) / l2
    if t < 0.0:
        return (p - v).length()
    elif t > 1.0:
        return (p - w).length()

    projection = v + (w - v).scaled(t)
    return (p - projection).length()


def distance_to_poly(point, poly):
    '''Return the distance to the nearest segment on the polygon.'''

    result = None
    N = len(poly)
    for i in range(N):
        this_segment = distance_to_segment(point, poly[i], poly[(i + 1) % N])
        if result is None or this_segment < result:
            result = this_segment

    return result
