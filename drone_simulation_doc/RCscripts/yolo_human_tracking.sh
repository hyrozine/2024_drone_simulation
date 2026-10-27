#!/bin/bash

cd ~/ROBOCUP2023/buptcReam/RCscripts

vehicle_num=0
while [ $vehicle_num -lt 6 ]
do
    python3 yolo_human_tracking.py typhoon_h480 $vehicle_num&
done