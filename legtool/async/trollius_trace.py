# Copyright 2014 Josh Pieper, jjp@pobox.com.
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

'''This module, when imported, hooks into the trollius exception
handling mechanism to ensure that even on python2, full stack trace
information is maintained as coroutines unwind.'''

import sys
import traceback

import trollius
import trollius as asyncio

# The following is an ugly ugly hack to get useful stack traces from
# coroutines with trollius on python 2.
old_future_set = trollius.Future.set_exception

def Future_set_exception(self, exc):
    tb = sys.exc_info()[2]
    if not hasattr(exc, '__frames__'):
        setattr(exc, '__frames__', [])

    frames = getattr(exc, '__frames__')
    if len(frames) == 0:
        frames.append(str(exc))
    frames[0:0] = traceback.format_tb(tb)

    old_future_set(self, exc)

    self._tb_logger.tb = frames

    self = None

trollius.Future.set_exception = Future_set_exception
