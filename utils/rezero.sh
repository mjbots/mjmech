#!/bin/bash

for device in $(seq 1 12); do
    echo $device;
    (echo "d rezero"
     sleep 0.5) | ($1 --type serial --serial_port /dev/ttyAMA0 --serial_baud 3000000 -t $device -c);
done
