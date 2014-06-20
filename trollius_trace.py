# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved

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
