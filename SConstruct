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

canonenv = Environment()
canonenv.Append(CPPFLAGS=['-Wall', '-Werror', '-g', '-std=c++1y'])

if ARGUMENTS.get('debug', 0):
    canonenv.Append(CPPFLAGS=['-O0'])
else:
    canonenv.Append(CPPFLAGS=['-O3'])

Export('canonenv')

SConscript(['imu/SConscript'], variant_dir='imu/build')
SConscript(['legtool/src/SConscript'], variant_dir='legtool/src/build',
           duplicate=0)
SConscript(['legtool/SConscript', 'scoring/manager/SConscript' ])
