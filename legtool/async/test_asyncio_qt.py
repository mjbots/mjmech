#!/usr/bin/python

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

'''A simple standalone manual test application to verify some of the
functionality of asyncio_qt.'''

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
