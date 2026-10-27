#!/bin/bash 

# 安装 terminator
# sudo apt install terminator

# 设定运行间隔时间
readonly time=2

# 设定路径
public_file_path="~"
private_file_path="~/ROBOCUP2023/buptcReam"


# 默认
terminator -e 'bash -ic "source ${private_file_path}/catkin_ws/devel/setup.bash ; roslaunch darknet_ros darknet_robocup.launch ; bash;"' -T "启动yolo" &
# tiny
# terminator -e 'bash -ic "source ${private_file_path}/catkin_ws/devel/setup.bash ; roslaunch darknet_ros darknet_robocup_tiny.launch ; bash;"' -T "启动yolo" &

sleep $time

terminator -e 'bash -ic "cd ${private_file_path}/RCscripts ; bash multi_gimbal_control.sh ; bash;" '  -T "启动云台控制" &
sleep $time

terminator -e 'bash -ic "cd ${private_file_path}/RCscripts ;
python3 yolo_human_tracking.py typhoon_h480 0 ;
python3 yolo_human_tracking.py typhoon_h480 1 ;
python3 yolo_human_tracking.py typhoon_h480 2 ;
python3 yolo_human_tracking.py typhoon_h480 3 ;
python3 yolo_human_tracking.py typhoon_h480 4 ;
python3 yolo_human_tracking.py typhoon_h480 5 ;
bash;"' -T "yolo_human_tracking" &
sleep $time

echo "Tracer"
terminator -e 'bash -ic "cd ${private_file_path}/RCscripts ; python3 Tracer.py; bash;"' -T "Tracer" &
sleep $time

terminator -e 'bash -ic "cd ${private_file_path}/RCscripts ;
python autoTakeoff.py ; 
sleep 0.2 ;
python autoTakeoff.py ; 
sleep 0.2 ;
python autoTakeoff.py ; 
sleep 0.2 ;
python autoTakeoff.py ; 
sleep 0.2 ;
python autoTakeoff.py ; 
sleep 0.2 ;
python autoTakeoff.py ; 
bash;"' -T "自动起飞" &
sleep $(($time + 6))

terminator -e 'bash -ic "cd ${private_file_path}/RCscripts ; python start_search.py ; bash;"' -T "start_search" &
sleep $time

echo "运行结束"