#!/usr/bin/python

import logging
import sys

import trollius as asyncio
from trollius import From

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

import asyncio_qt

logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)

@asyncio.coroutine
def simple_coroutine():
    while True:
        print "stuff"
        yield From(asyncio.sleep(2))

@asyncio.coroutine
def second_coroutine():
    while True:
        print "more!"
        yield From(asyncio.sleep(1.5))
        
def start_other():
    asyncio.Task(second_coroutine())
        
def main():
    asyncio.set_event_loop_policy(asyncio_qt.QtEventLoopPolicy())
        
    app = QtGui.QApplication(sys.argv)
    dialog = QtGui.QDialog()
    button = QtGui.QPushButton("text", dialog)
    button.clicked.connect(start_other)
    dialog.show()

    loop = asyncio.get_event_loop()
    asyncio.Task(simple_coroutine())

    loop.run_forever()

if __name__ == '__main__':
    main()
