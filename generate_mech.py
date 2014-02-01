#!/usr/bin/python

import math

CHASSIS_LENGTH = 0.2

PARAMETERS = {
    'chassis_mass' : 0.4,
    'chassis_length' : CHASSIS_LENGTH,
    'height' : 0.04,
    }

PREAMBLE = '''
<?xml version="1.0"?>
<sdf version="1.4">
  <model name="mj_mech">
    <static>false</static>

'''

SUFFIX = '''
  </model>
</sdf>
'''

CHASSIS = '''
    <link name="chassis">
      <pose>0 0 {height:f} 0 0 0</pose>
      <inertial><mass>{chassis_mass:f}</mass></inertial>
      <collision name="collision">
        <geometry><box><size>{chassis_length:f} {chassis_length:f} {height:f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{chassis_length:f} {chassis_length:f} {height:f}</size></box></geometry>
      </visual>
    </link>
'''

LINK = '''
    <link name="{name}">
      <pose>{x:f} {y:f} {z:f} {roll_rad:f} {pitch_rad:f} {yaw_rad:f}</pose>
      <inertial><mass>{mass:f}</mass></inertial>
      <collision name="collision">
        <geometry><box><size>{length:f} {width:f} {height:f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{length:f} {width:f} {height:f}</size></box></geometry>
      </visual>
    </link>
'''

JOINT = '''
    <joint type="revolute" name="{name}">
      <pose>{x:f} {y:f} {z:f} 0 0 0</pose>
      <child>{child}</child>
      <parent>{parent}</parent>
      <axis>
        <xyz>{axis_x:f} {axis_y:f} {axis_z:f}</xyz>
        <dynamics><damping>0.5</damping><friction>0.5</friction></dynamics>
      </axis>
    </joint>
'''

def merge(dict1, dict2):
    copy = dict1.copy()
    copy.update(dict2)
    return copy

COXA_LENGTH = 0.06
FEMUR_LENGTH = 0.06
TIBIA_LENGTH = 0.08

def main():
    print PREAMBLE

    print CHASSIS.format(**PARAMETERS)

    leg_x_sign = [ 1, 1, -1, -1]
    leg_y_sign = [ 1, -1, -1, 1]
    leg_yaw = [ 0, 0, math.pi, math.pi ]

    for leg_num in range(4):

        print LINK.format(
            **merge(PARAMETERS,
                    {'name' : 'coxa%d' % leg_num,
                     'x' : leg_x_sign[leg_num] * (0.5 * CHASSIS_LENGTH + 0.5 * COXA_LENGTH),
                     'y' : leg_y_sign[leg_num] * 0.5 * CHASSIS_LENGTH,
                     'z' : PARAMETERS['height'],
                     'mass' : 0.05,
                     'roll_rad' : 0,
                     'pitch_rad' : 0,
                     'yaw_rad' : leg_yaw[leg_num],
                     'length' : COXA_LENGTH,
                     'width' : 0.03,
                     'height' : 0.035}))
        print JOINT.format(
            **merge(PARAMETERS, {'name' : 'coxa_hinge%d' % leg_num,
                                 'x' : -0.5 * COXA_LENGTH,
                                 'y' : 0.0,
                                 'z' : 0.0,
                                 'child' : 'coxa%d' % leg_num,
                                 'parent' : 'chassis',
                                 'axis_x' : 0.0,
                                 'axis_y' : 0.0,
                                 'axis_z' : 1.0}))

        print LINK.format(
            **merge(PARAMETERS,
                    {'name' : 'femur%d' % leg_num,
                     'x' : leg_x_sign[leg_num] * (0.5 * CHASSIS_LENGTH + COXA_LENGTH + 0.5 * FEMUR_LENGTH),
                     'y' : leg_y_sign[leg_num] * 0.5 * CHASSIS_LENGTH,
                     'z' : PARAMETERS['height'],
                     'mass' : 0.05,
                     'roll_rad' : 0,
                     'pitch_rad' : 0,
                     'yaw_rad' : leg_yaw[leg_num],
                     'length' : FEMUR_LENGTH,
                     'width' : 0.035,
                     'height' : 0.030}))

        print JOINT.format(
            **merge(PARAMETERS,
                    {'name' : 'femur_hinge%d' % leg_num,
                     'x' : -0.5 * FEMUR_LENGTH,
                     'y' : 0.0,
                     'z' : 0.0,
                     'child' : 'femur%d' % leg_num,
                     'parent' : 'coxa%d' % leg_num,
                     'axis_x' : 0.0,
                     'axis_y' : leg_x_sign[leg_num] * 1.0,
                     'axis_z' : 0.0}))

        print LINK.format(
            **merge(PARAMETERS,
                    {'name' : 'tibia%d' % leg_num,
                     'x' : leg_x_sign[leg_num] * (0.5 * CHASSIS_LENGTH + COXA_LENGTH + FEMUR_LENGTH + 0.5 * TIBIA_LENGTH),
                     'y' : leg_y_sign[leg_num] * 0.5 * CHASSIS_LENGTH,
                     'z' : PARAMETERS['height'],
                     'mass' : 0.05,
                     'roll_rad' : 0.0,
                     'pitch_rad' : 0.0,
                     'yaw_rad' : leg_yaw[leg_num],
                     'length' : TIBIA_LENGTH,
                     'width' : 0.035,
                     'height' : 0.030}))

        print JOINT.format(
            **merge(PARAMETERS,
                    {'name' : 'tibia_hinge%d' % leg_num,
                     'x' : -0.5 * TIBIA_LENGTH,
                     'y' : 0.0,
                     'z' : 0.0,
                     'child' : 'tibia%d' % leg_num,
                     'parent' : 'femur%d' % leg_num,
                     'axis_x' : 0.0,
                     'axis_y' : leg_x_sign[leg_num] * 1.0,
                     'axis_z' : 0.0}))

    print SUFFIX

if __name__ == '__main__':
    main()
