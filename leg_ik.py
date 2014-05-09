# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import math

from tf import Point3D

class Configuration(object):
    coxa_min_deg = None
    coxa_idle_deg = None
    coxa_max_deg = None
    coxa_length_mm = None
    coxa_sign = 1
    coxa_ident = None

    femur_min_deg = None
    femur_idle_deg = None
    femur_max_deg = None
    femur_length_mm = None
    femur_sign = 1
    femur_ident = None

    tibia_min_deg = None
    tibia_idle_deg = None
    tibia_max_deg = None
    tibia_length_mm = None
    tibia_sign = 1
    tibia_ident = None

    servo_speed_dps = 360.0

class JointAngles(object):
    config = None
    coxa_deg = None # positive is rotating clockwise viewed from top
    femur_deg = None # positive is rotating upward
    tibia_deg = None # positive is rotating upward

    def command_dict(self):
        '''Return a dictionary mapping servo identifiers to commands
        in degrees.  This is the same format as the servo_controller
        module uses.'''
        return { self.config.coxa_ident : self.coxa_deg,
                 self.config.femur_ident : self.femur_deg,
                 self.config.tibia_ident : self.tibia_deg }

def lizard_3dof_ik(point_mm, config):
    '''Given a target end position in 3D coordinate space, return the
    required joint angles for a 3 degree of freedom lizard style
    leg.

    +y is away from the shoulder
    +x is clockwise from shoulder
    +z is up

    If no solution is possible, return None.
    '''
    # Solve for the coxa first, as it has only a single solution.
    coxa_deg = (config.coxa_sign *
                math.degrees(math.atan2(point_mm.x, point_mm.y)) +
                config.coxa_idle_deg)

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
    tibia_deg = (config.tibia_sign *
                 math.degrees(0.5 * math.pi - math.acos(tibia_cos)) +
                 config.tibia_idle_deg)

    if (tibia_deg < config.tibia_min_deg or
        tibia_deg > config.tibia_max_deg):
        return None

    # To solve for the femur angle, we first get the angle opposite
    # true_x, then the angle opposite the tibia.
    true_x_deg = math.degrees(math.atan2(true_x, -point_mm.z))

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

    femur_deg = (config.femur_sign * ((femur_im_deg + true_x_deg) - 90.0) +
                 config.femur_idle_deg)

    if (femur_deg < config.femur_min_deg or
        femur_deg > config.femur_max_deg):
        return None

    result = JointAngles()
    result.config = config
    result.coxa_deg = coxa_deg
    result.femur_deg = femur_deg
    result.tibia_deg = tibia_deg

    return result

class LizardIk(object):
    def __init__(self, config):
        self.config = config

    def do_ik(self, point_mm):
        return lizard_3dof_ik(point_mm, self.config)

    def worst_case_speed_mm_s(self, point_mm, direction_mm=None):
        '''Return the worst case linear velocity the end effector can
        achieve in the given orientation.'''
        step = 0.01
        nominal = self.do_ik(point_mm)

        if nominal is None:
            return None


        servo_step = step * self.config.servo_speed_dps
        result = None

        def update(result, advanced_servo_deg, nominal_servo_deg):
            if advanced_servo_deg == nominal_servo_deg:
                return

            this_speed = (servo_step /
                          abs(advanced_servo_deg - nominal_servo_deg))

            if result is None or this_speed < result:
                result = this_speed

            return result

        if direction_mm:
            normalized = direction_mm.scaled(1.0 / direction_mm.length())
            consider = [normalized.scaled(step)]
        else:
            consider = [Point3D(*val) for val in
                        (step, 0., 0.), (0., step, 0.), (0., 0., step)]

        for advance in consider:
            advanced = self.do_ik(point_mm + advance)

            if advanced is None:
                return None

            result = update(result, advanced.coxa_deg, nominal.coxa_deg)
            result = update(result, advanced.femur_deg, nominal.femur_deg)
            result = update(result, advanced.tibia_deg, nominal.tibia_deg)

        return result

    def servo_speed_dps(self):
        return self.config.servo_speed_dps

    def largest_change_deg(self, result1, result2):
        return max(abs(result1.coxa_deg - result2.coxa_deg),
                   abs(result1.femur_deg - result2.femur_deg),
                   abs(result1.tibia_deg - result2.tibia_deg))
