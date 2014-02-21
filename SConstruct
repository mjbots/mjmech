# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

env = Environment()

env.Append(CPPFLAGS=['-Wall', '-Werror', '-g'])
env.ParseConfig('pkg-config --cflags --libs gazebo')

SOURCES = [
    'servo_plugin.cc',
    ]

result = env.SharedLibrary('servo_plugin', SOURCES)

env.Command('mtool_main_window.py',
            'mtool_main_window.ui',
            'pyside-uic $SOURCE -o $TARGET')
