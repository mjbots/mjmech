import logging
import signal
import sys
import os
import traceback

sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../legtool/'))

import trollius as asyncio
import gbulb

# common helpers for vclient.py and vserver.py
g_quit_handlers = list()

def wrap_event(callback):
    """Wrap event callback so the app exit if it crashes"""
    def wrapped(*args, **kwargs):
        try:
            return callback(*args, **kwargs)
        except BaseException as e:
            logging.error("Callback %r crashed:", callback)
            logging.error(" %s %s" % (e.__class__.__name__, e))
            for line in traceback.format_exc().split('\n'):
                logging.error('| %s', line)
            for cb in g_quit_handlers:
                cb()
            raise
    return wrapped

@wrap_event
def _sigint_handler():
    # wrap_event decorator will make sure the exception stops event loop.
    raise Exception('Got SIGINT')

def asyncio_misc_init():
    asyncio.set_event_loop_policy(gbulb.GLibEventLoopPolicy())

    main_loop = asyncio.get_event_loop()
    main_loop.add_signal_handler(signal.SIGINT, _sigint_handler)
    g_quit_handlers.append(
        lambda:  main_loop.call_soon_threadsafe(main_loop.stop))
