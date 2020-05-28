#!/usr/bin/python3 -B

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

import enum
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def main():
    tvals = np.arange(0, 50, 0.01)
    theta0 = 0
    w = 0.1
    vx = 3.0
    vy = 4.0

    def evalx(t):
        return (vx * math.sin(t * w) +
                vy * math.cos(t * w) - vy) / w
    def evaly(t):
        return (vy * math.sin(t * w) -
                vx * math.cos(t * w) + vx) / w

    plt.plot([evalx(t) for t in tvals],
             [evaly(t) for t in tvals])
    plt.show()


if __name__ == '__main__':
    main()
