# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

canonenv = Environment()
canonenv.Append(CPPFLAGS=['-Wall', '-Werror', '-g', '-std=c++1y'])
#canonenv.ParseConfig('pkg-config --cflags --libs gazebo')
Export('canonenv')

SConscript(['SConscript', 'legtool/SConscript'])
SConscript(['imu/SConscript'], variant_dir='imu/build')
