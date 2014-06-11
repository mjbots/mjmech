# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import errno
import thread
import trollius as asyncio
from trollius import From, Return
import logging
import socket
import sys
import types

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

logger = logging.getLogger(__name__)

class QtEventLoopPolicy(asyncio.AbstractEventLoopPolicy):
    def __init__(self):
        self.loops = {}
    
    def get_event_loop(self):
        ident = thread.get_ident()
        if ident not in self.loops:
            self.loops[ident] = QtEventLoop()
        return self.loops[ident]

    def set_event_loop(self, loop):
        ident = thread.get_ident()
        self.loops[ident] = loop

    def new_event_loop(self):
        return QtEventLoop()

class _Invoker(QtCore.QObject):
    signal = QtCore.Signal(types.FunctionType)

    def __init__(self):
        super(_Invoker, self).__init__()
        self.signal.connect(self._run, QtCore.Qt.QueuedConnection)

    def schedule(self, callback):
        self.signal.emit(callback)

    def _run(self, callback):
        callback()
    

class QtEventLoop(asyncio.AbstractEventLoop):
    def __init__(self):
        self._is_running = False
        self._readers = {}
        self._writers = {}
        self._exceptions = {}
        self._exception_handler = None
        self._invoker = _Invoker()
        
        
    def run_forever(self):
        self._is_running = True
        QtCore.QCoreApplication.exec_()

    def run_until_complete(self, future):
        def stop(future):
            QtCore.QtCoreApplication.exit(0)

        future.add_done_callback(stop)
        try:
            self._is_running = True
            QtCore.QtCoreApplication.exec_()
        finally:
            self._is_running = False
            future.remove_done_callback(stop)

    def is_running(self):
        return self._is_running

    def stop(self):
        self.call_soon(QtCore.QtCoreApplication.exit, 0)

    def close(self):
        assert not self._is_running
        # What to do here?

    def call_soon(self, callback, *args):
        self._invoker.schedule(lambda: callback(*args))

    def call_soon_threadsafe(self, callback, *args):
        raise NotImplementedError

    def call_later(self, delay, callback, *args):
        result = asyncio.Handle(callback, args, self)
        QtCore.QTimer.singleShot(int(delay * 1000.0), lambda: result._run())
        return result

    def call_at(self, when, callback, *args):
        delay = max(0., when - time.time())
        return self.call_later(delay, callback, *args)

    def time(self):
        return time.time()

    class _Notifier(object):
        def __init__(self, qnotifier, callback, args):
            self.qnotifier = qnotifier
            self.callback = callback
            self.args = args

        def call(self):
            self.callback(*self.args)

    def add_reader(self, fd, callback, *args):
        self._add_notifier(QtCore.QSocketNotifier.Read, self._readers,
                          fd, callback, args)

    def _add_notifier(self, notifier_type, mapping,
                      fd, callback, args):
        if fd in mapping:
            notifier = mapping[fd]
            notifier.callback = callback
            notifier.args = args
            notifier.qnotifier.setEnabled(True)
        else:
            notifier = self._Notifier(
                QtCore.QSocketNotifier(
                    fd, notifier_type),
                callback, args)
            mapping[fd] = notifier
            notifier.qnotifier.activated.connect(notifier.call)

    def remove_reader(self, fd):
        assert fd in self._readers
        self._readers[fd].qnotifier.setEnabled(False)

    def add_writer(self, fd, callback, *args):
        self._add_notifier(QtCore.QSocketNotifier.Write, self._writers,
                           fd, callback, args)

    def remove_writer(self, fd):
        assert fd in self._writers
        self._writers[fd].qnotifier.setEnabled(False)

    def add_exception(self, fd, callback, *args):
        self._add_notifier(QtCore.QSocketNotifier.Exception, self._exceptions,
                           fd, callback, args)

    def remove_exception(self, fd):
        assert fd in self._exceptions
        self._exceptions[fd].qnotifier.setEnabled(False)

    @asyncio.coroutine
    def sock_recv(self, sock, nbytes):
        assert sock.gettimeout() == 0.0
        event = asyncio.Event()
        try:
            self.add_reader(sock.fileno(), event.set)
            yield From(event.wait())
        finally:
            self.remove_reader(sock.fileno())

        data = sock.recv(nbytes)
        raise Return(data)

    @asyncio.coroutine
    def sock_sendall(self, sock, data):
        assert sock.gettimeout() == 0.0
        event = asyncio.Event()
        to_send = data[:]
        try:
            self.add_writer(sock.fileno(), event.set)
            while len(to_send):
                yield From(event.wait())
                event.clear()
                written = sock.send(to_send)
                to_send = to_send[written:]
        finally:
            self.remove_writer(sock.fileno())

    @asyncio.coroutine
    def sock_connect(self, sock, address):
        assert sock.gettimeout() == 0.0

        try:
            result = sock.connect(address)
            raise Return()
        except socket.error as e:
            if e.args[0] != errno.EINPROGRESS:
                raise e

            pass  # ignore, as we need to wait

        # Nope, now we have to wait.
        event = asyncio.Event()
        try:
            self.add_writer(sock.fileno(), event.set)
            self.add_exception(sock.fileno(), event.set)
            yield From(event.wait())
        finally:
            self.remove_writer(sock.fileno())
            self.remove_exception(sock.fileno())

        error = sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
        if error != 0:
            # TODO jpieper: This doesn't seem to result in a usable
            # error message.
            raise socket.error(error)

        raise Return()

    @asyncio.coroutine
    def sock_accept(self, sock):
        print 'sock_accept:', sock, sock.fileno()
        assert sock.gettimeout() == 0.0

        while True:
            event = asyncio.Event()
            try:
                self.add_reader(sock.fileno(), event.set)
                yield From(event.wait())
            finally:
                self.remove_reader(sock.fileno())

            print 'sock_accept reader ready!'
            try:
                result = sock.accept()
                result[0].setblocking(False)
                print "got accept!:", result
                raise Return(result)
            except socket.error as e:
                if (e.args[0] != errno.EINPROGRESS and
                    e.args[0] != errno.EAGAIN):
                    raise e

                pass  # ignore, as we need to wait


    @asyncio.coroutine
    def getaddrinfo(self, host, port, family=0, type=0, proto=0, flags=0):
        raise NotImplementedError

    @asyncio.coroutine
    def getnameinfo(self, sockaddr, flags=0):
        raise NotImplementedError

    def get_debug(self):
        return True

    def set_exception_handler(self, handler):
        self._exception_handler = handler

    def default_exception_handler(self, context):
        message = context.get('message')
        if not message:
            message = 'Unhandled exception in event loop'

        exception = context.get('exception')
        if exception is not None:
            if hasattr(exception, '__traceback__'):
                # Python 3
                tb = exception.__traceback__
            else:
                # call_exception_handler() is usually called indirectly
                # from an except block. If it's not the case, the traceback
                # is undefined...
                tb = sys.exc_info()[2]
            exc_info = (type(exception), exception, tb)
        else:
            exc_info = False

        log_lines = [message]
        for key in sorted(context):
            if key in ('message', 'exception'):
                continue
            log_lines.append('{0}: {1!r}'.format(key, context[key]))

        logger.error('\n'.join(log_lines), exc_info=exc_info)

    def call_exception_handler(self, context):
        if self._exception_handler is None:
            try:
                self.default_exception_handler(context)
            except Exception:
                # Second protection layer for unexpected errors
                # in the default implementation, as well as for subclassed
                # event loops with overloaded "default_exception_handler".
                logger.error('Exception in default exception handler',
                             exc_info=True)
        else:
            try:
                self._exception_handler(self, context)
            except Exception as exc:
                # Exception in the user set custom exception handler.
                try:
                    # Let's try default handler.
                    self.default_exception_handler({
                        'message': 'Unhandled error in exception handler',
                        'exception': exc,
                        'context': context,
                    })
                except Exception:
                    # Guard 'default_exception_handler' in case it's
                    # overloaded.
                    logger.error('Exception in default exception handler '
                                 'while handling an unexpected error '
                                 'in custom exception handler',
                                 exc_info=True)
        
