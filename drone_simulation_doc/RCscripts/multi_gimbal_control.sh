#!/bin/bash

vehicle_num=0
while [ $vehicle_num -lt 6 ]
do
    python gimbal_control.py typhoon_h480 $vehicle_num &
    let "vehicle_num++"
    sleep 0.1
done