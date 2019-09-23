#!/bin/bash

for device in 3 6 9 12; do
    echo $device
    (echo "conf set servo.pid_position.kp 2200";
     sleep 0.05;
     echo "conf set servo.pid_position.kd 9";
     sleep 0.2;
     echo "conf write";
     sleep 0.5) | ($1 -t $device -c)
done
