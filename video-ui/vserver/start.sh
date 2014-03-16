#!/bin/bash

LPID=/tmp/vserver-loop.pid
if test -f $LPID  && kill -0 $(cat $LPID); then
    echo Loop is running, will use it instead
    vs_pid=$(pgrep vserver.py)
    if [[ -f /tmp/vserver.log ]]; then
        mv -f /tmp/vserver.log /tmp/vserver.log.1
    fi
    if [[ "$vs_pid" != "" ]]; then
        echo old process: $(ps --no-headers -l -p $vs_pid)
        kill $vs_pid
        for x in `seq 30 -1 0`; do
            if ! kill -0 $vs_pid 2>/dev/null; then break; fi
            echo Waiting for process to die
            sleep 0.2
        done
        if [[ "$x" == 0 ]]; then
            echo Process failed to exit
            exit 1
        fi
    else
        echo No process to kill
    fi
    for x in `seq 50 -1 0`; do
        vs_pid=$(pgrep vserver.py)
        if [[ "$vs_pid" != "" ]]; then break; fi
        if [[ -f /tmp/vserver.log ]]; then
            # We got the log, but no process -- this means process failed to start.
            break
        fi
        echo Waiting for process to start
        sleep 1
    done
    if [[ "$vs_pid" == "" ]]; then
        echo new process failed to start
        EC=1
    else
        echo new process: $(ps --no-headers -l -p $vs_pid)
        EC=0
    fi
    sleep 1 # give server time to start
    cat -n /tmp/vserver.log
    exit $EC
fi

echo No loop found, starting here
cd $(dirname $(readlink -f "$0")) || exit 1
./vserver.py