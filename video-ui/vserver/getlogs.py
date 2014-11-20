#!/usr/bin/python
"""A simple program to read the logs from vserver_launcher.py
"""

import sys
import os
import optparse
import time
import errno

from vserver_launcher import LOG_NAME, PID_NAME

def open_logfile():
    try:
        return open(LOG_NAME, 'r')
    except IOError as e:
        if e.errno != errno.ENOENT:
            raise
        # File not found
        return None

def is_alive():
    return os.path.exists(PID_NAME)

def print_last_lines(logfile, num_lines):
    # Seek to start of file. This will always succeed.
    logfile.seek(0, os.SEEK_SET)

    # Seek back some distance from the end. Assume no line is longer than 120
    # characters. This will fail if file is too short.
    try:
        logfile.seek(-max(0, num_lines * 120), os.SEEK_END)
    except IOError:
        pass

    # Read data and print last lines
    data = logfile.readlines()
    if num_lines > 0:
        for line in data[-num_lines:]:
            print line.rstrip()
        sys.stdout.flush()

def say(msg):
    print '***', msg, '***'
    sys.stdout.flush()

def main(opts):
    logfile = open_logfile()

    if not logfile:
        say('No logfile found. Is vserver_launcher running?')
        return 1

    if (opts.last_crash > 0) and not is_alive():
        say('Program is dead. Last logs:')
        print_last_lines(logfile, opts.last_crash)
    elif opts.last > 0:
        say('Last logs:')
        print_last_lines(logfile, opts.last)

    if opts.restart and logfile:
        say('Requesting vserver.py restart')
        os.unlink(LOG_NAME)
        logfile.close()
        for _ in xrange(20):
            logfile = open_logfile()
            if logfile:
                break
            time.sleep(0.25)
        else:
            say('Logfile did not appear. Is vserver_launcher running?')
            return 2

    stop_at = None
    if (opts.wait_exit is not None) and (opts.wait_exit >= 0):
        stop_at = time.time() + opts.wait_exit
    last_msg = time.time()
    say("Live logs")

    log_inode = None
    # Do the tail
    while True:
        if logfile is not None:
            block = logfile.read(1024)
        else:
            # wait for file to appear
            block = ''
            logfile = open_logfile()
            if logfile:
                say('File re-opened')
        if block != '':
            sys.stdout.write(block)
            sys.stdout.flush()
            last_msg = time.time()
            continue
        if not is_alive():
            say('Program has exited. Terminating')
            return 1
        if (stop_at is not None) and (stop_at < time.time()):
            break
        if opts.idle_marker > 0 and (
            (last_msg + opts.idle_marker) < time.time()):
            say('Program still running, no messages')
            last_msg = time.time()

        # Sleep
        time.sleep(opts.poll_time)
        if logfile is not None:
            # Reset EOF marker if we have the file
            logfile.seek(0, os.SEEK_CUR)
            # Check if file was replaced
            if log_inode is None:
                log_inode = os.fstat(logfile.fileno()).st_ino
            try:
                disk_inode = os.stat(LOG_NAME).st_ino
            except OSError:
                disk_inode = -1
            if log_inode != disk_inode:
                say('Logfile disappeared')
                logfile.close()
                logfile = None
                log_inode = None


    say('Time expired, program still running')
    return 0

if __name__ == '__main__':
    parser = optparse.OptionParser(usage='%prog [args]')
    parser.add_option('--wait-exit', '-w', type='float', metavar='SEC',
                      help='If program has not exited after that many seconds,'
                      'exit with code 0')
    parser.add_option('--last', '-n', type='int', metavar='LINES', default=30,
                      help='Print that many last log lines (default %default)')
    parser.add_option('--last-crash', '-C', type='int', metavar='LINES',
                      default=30,
                      help='If program has crashed, print that many log lines '
                      '(default %default)')
    # The reason for idle marker is process cleanup -- if ssh without -t
    # option is interrupted, the process will stay until next print statement;
    # if vserver.py is not writing anything, this will happen at the next idle
    # marker.
    parser.add_option('--idle-marker', '-I', type='float', metavar='SEC',
                      default=60,
                      help='If no logs are coming, print a marker every SEC '
                      'seconds (default %default, 0 to disable)')
    parser.add_option('--restart', action='store_true',
                      help='Tell vserver_launcher to start/restart app')
    parser.add_option('--poll-time', type='float', default=0.25, metavar='SEC',
                      help='How frequently to poll for new data (def %default)')
    opts, args = parser.parse_args()
    if args:
        parser.error('No positional arguments accepted')
    sys.exit(main(opts))
