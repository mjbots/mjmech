# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

canonenv = Environment()
canonenv.Append(CPPFLAGS=['-Wall', '-Werror', '-g'])
#canonenv.ParseConfig('pkg-config --cflags --libs gazebo')
Export('canonenv')

SConscript(['SConscript', 'legtool/SConscript'])

