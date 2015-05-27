# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

canonenv = Environment()
canonenv.Append(CPPFLAGS=['-Wall', '-Werror', '-g', '-std=c++1y'])

if ARGUMENTS.get('debug', 0):
    canonenv.Append(CPPFLAGS=['-O0'])
else:
    canonenv.Append(CPPFLAGS=['-O3'])

#canonenv.ParseConfig('pkg-config --cflags --libs gazebo')
Export('canonenv')

SConscript(['SConscript', 'legtool/SConscript', 'scoring/manager/SConscript'])
SConscript(['imu/SConscript'], variant_dir='imu/build')
