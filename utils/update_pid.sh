#!/bin/bash

for device in 1 2 4 5 7 8 10 11; do
    echo $device
    (echo "conf set servo.pid_position.kp 1300";
     sleep 0.05;
     echo "conf set servo.pid_position.kd 9";
     sleep 0.2;
     echo "conf write";
     sleep 0.5) | ($1 -t $device -c)
done
