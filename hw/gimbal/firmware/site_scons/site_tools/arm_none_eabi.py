# Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

TOOLS = {
    'CC' : 'gcc',
    'CXX' : 'g++',
    'LD' : 'gcc',
    'AR' : 'ar',
    'AS' : 'gcc',
    'OBJCOPY' : 'objcopy',
    'OBJDUMP' : 'objdump',
    }


def generate(env, **kwargs):
    # Let's assume that the host version of the compiler is here and
    # available.
    gnu_tools = ['gcc', 'g++', 'gnulink', 'ar', 'gas']
    for tool in gnu_tools:
        env.Tool(tool)

    for key, value in TOOLS.iteritems():
        env[key] = 'arm-none-eabi-' + value

    env.Append(ASFLAGS=['-c'])
    env['PROGSUFFIX'] = '.elf'


def exists(env):
    return 1
