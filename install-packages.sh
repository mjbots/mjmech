#!/bin/sh
set -ex
apt-get install python-eventlet python-pyside pyside-tools python-pygame \
    python-pip scons avr-libc \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
pip install --pre pygazebo
pip install --pre trollius
pip install enum34
