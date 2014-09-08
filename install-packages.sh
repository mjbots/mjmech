#!/bin/sh
set -ex
apt-get install python-eventlet python-pyside scons python-pygame pyside-tools
pip install --pre pygazebo
pip install --pre trollius
pip install enum34
