# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

import math

class FilterConstant(object):
    '''These are the current best known constants.'''

    process_noise_gyro = math.radians(0.0002) ** 2
    process_noise_bias = math.radians(0.0256) ** 2
    measurement_noise_accel = 4.0 ** 2
    initial_noise_attitude = 3e-2 ** 2
    initial_noise_bias = math.radians(0.05) ** 2
