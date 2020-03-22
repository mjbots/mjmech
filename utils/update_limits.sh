#!/bin/bash

for device in 1 2 3 4 5 6 7 8 9 10 11 12; do
    echo $device
    (echo "conf set servopos.position_min -0.7";
     sleep 0.05;
     echo "conf set servopos.position_max 0.7";
     sleep 0.2;
     echo "conf write";
     sleep 0.5) | ($1 -t $device -c)
done
