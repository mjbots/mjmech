# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import math

class Point3D(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '<Point3D x/y/z %r/%r/%f>' % (self.x, self.y, self.z)

    def __add__(self, other):
        return Point3D(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z)

    def __mul__(self, other):
        return Point3D(
            self.x * other.x,
            self.y * other.y,
            self.z * other.z)

class Configuration(object):
    coxa_min_deg = None
    coxa_max_deg = None
    coxa_length_mm = None

    femur_min_deg = None
    femur_max_deg = None
    femur_length_mm = None

    tibia_min_deg = None
    tibia_max_deg = None
    tibia_length_mm = None

class JointAngles(object):
    coxa_deg = None
    femur_deg = None # positive is rotating upward
    tibia_deg = None # positive is rotating upward

def lizard_3dof_ik(point_mm, config):
    '''Given a target end position in 3D coordinate space, return the
    required joint angles for a 3 degree of freedom lizard style
    leg.

    If no solution is possible, return None.
    '''
    # Solve for the coxa first, as it has only a single solution.
    coxa_deg = math.degrees(math.atan2(point_mm.y, point_mm.x))

    if (coxa_deg < config.coxa_min_deg or
        coxa_deg > config.coxa_max_deg):
        return None

    # x-coordinate of femur/tibia pair after rotating to 0 coxa
    true_x = (math.sqrt(point_mm.x ** 2 + point_mm.y ** 2) -
              config.coxa_length_mm)
    im = math.sqrt(point_mm.z ** 2 + true_x ** 2)


    # The new femur/tibia pair makes a triangle where the 3rd side is
    # the hypotenuse of the right triangle composed of z and im, lets
    # call it c.
    #
    #           --\  femur
    #           |\ --\
    #           | \   --\
    #           |  --    |
    #          z|  im\   | tibia
    #           |     --\|
    #           ----------
    #            true_x
    #
    # im = math.sqrt(z ** 2 + true_x ** 2)
    #
    # Then, we can use the law of cosines to find the angle opposite
    # im, which is the angle between the femur and tibia.
    #
    # im ** 2 = a ** 2 + b ** 2 + 2 * a * b * cos(C)
    #
    # Solving for C yields:
    #
    #  C = acos((im ** 2 - a ** 2 - b ** 2) / (2 * a * b))

    tibia_cos = ((im ** 2 -
                  config.tibia_length_mm ** 2 -
                  config.femur_length_mm ** 2) /
                 (2 * config.tibia_length_mm * config.femur_length_mm))
    if tibia_cos < -1.0 or tibia_cos > 1.0:
        return None

    # For our purposes, a 0 tibia angle should equate to a right angle
    # with the femur, so subtract off 90 degrees.
    tibia_deg = math.degrees(0.5 * math.pi - math.acos(tibia_cos))

    if (tibia_deg < config.tibia_min_deg or
        tibia_deg > config.tibia_max_deg):
        return None

    # To solve for the femur angle, we first get the angle opposite
    # true_x, then the angle opposite the tibia.
    true_x_deg = math.degrees(math.atan2(true_x, point_mm.z))

    # Then the angle opposite the tibia is also found the via the law
    # of cosines.
    #
    #  tibia ** 2 = femur ** 2 + im ** 2 + 2 * femur * im * cos(femur_im)
    #
    #  femur_im = acos ( (tibia ** 2 - im ** 2 - femur ** 2) /
    #                    (2 * femur * im) )

    femur_im_cos = -(config.tibia_length_mm ** 2 -
                     config.femur_length_mm ** 2 -
                     im ** 2) / (2 * config.femur_length_mm * im)
    if femur_im_cos < -1.0 or femur_im_cos > 1.0:
        return None

    femur_im_deg = math.degrees(math.acos(femur_im_cos))

    femur_deg = (femur_im_deg + true_x_deg) - 90.0

    if (femur_deg < config.femur_min_deg or
        femur_deg > config.femur_max_deg):
        return None

    result = JointAngles()
    result.coxa_deg = coxa_deg
    result.femur_deg = femur_deg
    result.tibia_deg = tibia_deg

    return result
