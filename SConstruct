# Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import subprocess

env = Environment()

# Set up our global environment.
if ARGUMENTS.get('debug', 0):
    env.Append(CPPFLAGS=['-O0'])
else:
    env.Append(CPPFLAGS=['-O3'])

env.Append(CPPPATH=['#/'])
env.Append(CPPFLAGS=['-Wall', '-Werror', '-g', '-std=c++1y'])
env.Append(LINKFLAGS=['-rdynamic'])
env.Append(LIBS=['snappy',
                 'boost_system',
                 'boost_program_options',
                 'pthread',
                 'rt',
                 'log4cpp',
                 'boost_filesystem',
                 'boost_date_time'])

env.ParseConfig('pkg-config --cflags --libs eigen3')
env.ParseConfig('pkg-config --cflags --libs opencv')

env['UBUNTU_RELEASE'] = subprocess.check_output(["lsb_release", "-cs"]).strip()

canonenv = env
Export('canonenv')

import os
variant_suffix = '-' + os.uname()[4]

subdirs = ['base', 'mech', 'python', 'legtool', 'tools']

if not os.uname()[4].startswith('arm') and env['UBUNTU_RELEASE'] != 'xenial':
    subdirs += ['simulator']

for subdir in subdirs:
    SConscript(subdir + '/SConscript',
               variant_dir=subdir + '/build' + variant_suffix,
               duplicate=0)
