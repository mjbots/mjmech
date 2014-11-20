#!/usr/bin/python
"""
This is vserver_launcher.py, which is automatically launched on startup via
upstart file vserver-launcher.conf.

This is the only file permanently installed into flash, and thus it should
have as few dependencies as possible. It should not log too much stuff either
-- stdout/stderr go to upstart log which is not rotated.

It has two functions:
(1) broadcast packets with current hostname so vclient can locate the robot
(2) Launch/re-launch the main script as following:
 - script uses a single 'log' file (LOG_NAME below)
 - on startup, 'log' is created with bogus content
 - script checks is 'log' disappeared. If it did, script will:
  - launch START_SH script with stdout/stderr redirected


"""


import socket
import subprocess
import time
import sys
import optparse
import os
import json

g_start_time = time.time()
g_main_loop = None
gobject = None  # module ref

# log file. App is [re-]started when this file is removed.
LOG_NAME = '/tmp/vserver.log'
# PID file. It contains app's PID. It is removed if app exits.
# It is created before logfile is created (but it may be empty for some time)
PID_NAME = '/tmp/vserver.pid'
# main file to launch
START_SH = 'exec /tmp/vserver/start.sh'

class UdpAnnouncer(object):
    PORT = 13355
    INTERVAL = 0.5

    def __init__(self):
        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setblocking(0)

        #self.logger.info('Binding to port %d' % self.PORT)
        #self.sock.bind(('', self.PORT))
        print >>sys.stderr, (
            'Will broadcast on port %d every %.2f seconds' %
            (self.PORT, self.INTERVAL))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.dest = ('255.255.255.255', self.PORT)
        self.seq = 0
        self.proc = None
        self.running_since = None
        self.last_error = None

        # Create LOG_NAME to prevent app autostart on boot
        with open(LOG_NAME, 'a') as f:
            print >>f, '<fresh start>'

        # Remove PID file if exists
        try: os.unlink(PID_NAME)
        except (IOError, OSError): pass

        gobject.timeout_add(int(self.INTERVAL * 1000),
                            self._on_timeout_wrapped)

    def _on_timeout_wrapped(self):
        try:
            return self._on_timeout()
        except:
            g_main_loop.quit()
            raise

    def _on_timeout(self):
        # Check if process has died
        if self.proc and (self.proc.poll() is not None):
            rv = self.proc.wait()
            self.proc = None
            self.running_since = None
            print >>sys.stderr, 'Process died: %s' % rv
            os.unlink(PID_NAME)
            # Log is still around, so we will not start a new one.

        if os.path.exists(LOG_NAME):
            # Log exists, no need to do anything
            pass
        elif self.proc is not None:
            # Log disappeared, but we have a process. Kill it.
            # (we will send a signal every second, and will re-start it once
            # it dies)
            try:
                self.proc.terminate()
                print >>sys.stderr, 'Terminated process'
            except Exception as e:
                print >>sys.stderr, 'Failed to terminate process: %s' % e
        else:
            # No log, no process -- start one

            # Create PID file (do it befor LOG_NAME, so a watcher cannot find
            # log and no PID)
            pidfile = open(PID_NAME, 'w')

            # We are not using spawn_async because it cannot redirect output to
            # file.
            logfile = open(LOG_NAME, 'w')
            # We pass shell=True to get text message instead of exception in
            # case of non-executable or missing file.
            self.proc = subprocess.Popen(
                START_SH, shell=True, stdout=logfile, stderr=subprocess.STDOUT)
            print >>pidfile, self.proc.pid
            print >>sys.stderr, 'Process started, pid %d' % self.proc.pid
            self.running_since = time.time()
            # Logfiles/pidfiles are no longer needed
            logfile.close()
            pidfile.close()

        # check if process has exited
        # send announcement
        self.seq += 1
        msg = json.dumps({
                'type': 'announce',
                'seq': self.seq,
                # We do not know which port is used by control interface.
                #'cport': None,
                # Main app start time (or None)
                'running_since': self.running_since,
                # Launcher start time
                'start_time': g_start_time,
                'host': os.uname()[1]})

        try:
            self.sock.sendto(msg + '\n', self.dest)
            error = 'success'
        except socket.error as e:
            error = str(e)

        if error != self.last_error:
            self.last_error = error
            # Log only once per network error
            print >>sys.stderr, 'Broadcast result: %s' % error

        return True

def main(opts):
    # Only import gobject when we start, not when getlogs imports us.
    # (othewise, getlogs.py will not exit on Ctrl+C properly)
    from gi.repository import GObject
    global gobject, g_main_loop
    gobject = GObject
    g_main_loop = gobject.MainLoop()

    ann = UdpAnnouncer()
    g_main_loop.run()

if __name__ == '__main__':
    parser = optparse.OptionParser()
    opts, args = parser.parse_args()
    if args:
        parser.error('No args accepted')
    main(opts)
