import logging
import signal
import sys
import os
import traceback
import time

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

@wrap_event
def _sigterm_handler():
    # wrap_event decorator will make sure the exception stops event loop.
    raise Exception('Got SIGTERM')

def asyncio_misc_init():
    asyncio.set_event_loop_policy(gbulb.GLibEventLoopPolicy())

    main_loop = asyncio.get_event_loop()
    main_loop.add_signal_handler(signal.SIGINT, _sigint_handler)
    main_loop.add_signal_handler(signal.SIGTERM, _sigterm_handler)
    g_quit_handlers.append(
        lambda:  main_loop.call_soon_threadsafe(main_loop.stop))

def logging_init(verbose=True):
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    # Code below is like basicConfig, but we do not apply limits on loggers;
    # instead we apply them on handlers.
    outhandler = logging.StreamHandler()
    outhandler.setFormatter(
        logging.Formatter(
            fmt=("%(asctime)s.%(msecs).3d [%(levelname).1s]"
                    " %(name)s: %(message)s"),
            datefmt="%T"))
    root.addHandler(outhandler)
    if not verbose:
        outhandler.setLevel(logging.INFO)

class MemoryLoggingHandler(logging.Handler):
    """Handler that just appends data to python array.
    The elements are tuples:
       (time, level, logger_name, message)
    """
    SHORT_LEVEL_NAMES = {
        logging.CRITICAL: 'C',
        logging.ERROR: 'E',
        logging.WARNING: 'W',
        logging.INFO: 'I',
        logging.DEBUG: 'D',
        }

    def __init__(self, install=False, max_records=10000):
        logging.Handler.__init__(self)
        self.data = list()
        self.max_records = max_records
        self.on_record = list()
        self.last_time = 0
        if install:
            logging.getLogger().addHandler(self)

    def emit(self, record):
        """Part of logging.Handler interface"""
        ts = record.created
        if ts <= self.last_time:
            # timestamp must always increase
            ts = self.last_time + 1.0e-6
        self.last_time = ts
        self.data.append(
            (ts,
             record.levelno,
             record.name,
             record.getMessage()))
        while len(self.data) > self.max_records:
            self.data.pop(0)
        for cb in self.on_record:
            cb()

    @staticmethod
    def to_dict(mtuple, time_field='time'):
        """Given a 4-tuple, convert it to dict"""
        return {
            time_field: mtuple[0],
            'levelno': mtuple[1],
            'name': mtuple[2],
            'message': mtuple[3]}

    @classmethod
    def to_string(cls, mtuple):
        """Given a 4-tuple, convert it to string (default formatted)
        """
        return "%s [%s] %s: %s" % (
            time.strftime("%T", time.localtime(mtuple[0])),
            cls.SHORT_LEVEL_NAMES.get(mtuple[1], mtuple[1]),
            mtuple[2], mtuple[3])

    @staticmethod
    def relog(mtuple, delta_t=0, prefix=''):
        """Given a 4-tuple, re-log it to local logger"""
        # NOTE: this igores whole logger hierarchy. If we ever use it, pass a
        # name here.
        root = logging.getLogger()
        assert len(mtuple) == 4
        rec = root.makeRecord(
            prefix + mtuple[2], mtuple[1], 'remote-file', -1, mtuple[3],
            [], None, 'remote-func', None)
        # Override time. There is no better way.
        ct = delta_t + mtuple[0]
        rec.created = ct
        rec.msecs = (ct - long(ct)) * 1000
        rec.relativeCreated = (rec.created - logging._startTime) * 1000
        # Dispatch.
        root.handle(rec)
