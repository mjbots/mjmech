#!/bin/bash

RESTPOS=(NAN 0.25 0 0 -0.25 0 0 -0.25 0 0 0.25 0 0)

for device in $(seq 1 12); do
    echo $device;
    (echo "d rezero ${RESTPOS[$device]}"
     sleep 0.5) | ($1 -t $device -c);
done
