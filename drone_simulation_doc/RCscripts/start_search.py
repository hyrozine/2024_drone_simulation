# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist,PoseStamped, Pose
import sys, select, os
import tty, termios
from std_msgs.msg import String, Int8
import numpy as np
import sys
# sys.path.append('..')

"""
这个文件的用于初始(还没视野)时的运动规划
"""

mode = 0
# 0 六架默认
# 1 六架靠外
# 2 三架默认

err_threshold = 0.5
UAVpos=np.zeros([6,3], dtype=float)
goal_pub = [None]*6; goal_point = [None]*6
current_pose = [None]*6
override_subscriber = []
isOverride = [0] * 6
lastOverride = [0] * 6
leave_route = [0] * 6
no_buffer_location = [1] * 6

def local_pose_callback(data,id): # 计算出当前位置，在状态机为0时发布新目标点
    UAVpos[id][0] = data.pose.position.x + bias[id][0]
    UAVpos[id][1] = data.pose.position.y + bias[id][1]
    UAVpos[id][2] = data.pose.position.z + bias[id][2]
    if isOverride[UAV_id] == 0 :
        global targetPoses
        targetPoses=goal_publisher(id,targetPoses)

def setpoint(UAV_id,targetPoses):
    goal_point[UAV_id]=Pose()        #初始化消息类型
    goal_point[UAV_id].position.x=float(targetPoses[UAV_id][0][0] - bias[UAV_id][0])        #无人机目标位置
    goal_point[UAV_id].position.y=float(targetPoses[UAV_id][0][1] - bias[UAV_id][1])
    goal_point[UAV_id].position.z=float(targetPoses[UAV_id][0][2] - bias[UAV_id][2])
    goal_point[UAV_id].orientation.x=0          #四元数
    goal_point[UAV_id].orientation.y=0
    goal_point[UAV_id].orientation.z=0
    goal_point[UAV_id].orientation.w=1  
    #goal_point[UAV_id].header.stamp = rospy.Time.now()#得到新目标的时间
    goal_pub[UAV_id].publish(goal_point[UAV_id]) 
    # print(str(goal_point[UAV_id].position.x) +" "+ str(goal_point[UAV_id].position.y)+" "+str(goal_point[UAV_id].position.z))

def CheckState(UAV_id,targetPoses,posx,posy):#如果到达目标位置就发布新的目标
    if isOverride[UAV_id] == 1:
        return targetPoses
    if ((targetPoses[UAV_id][0][0]-posx)**2+(targetPoses[UAV_id][0][1]-posy)**2)<err_threshold**2:
        # print("UAV%d arrived!"%(UAV_id))
        # print("--------------------------------YAY---------------------------------------" )
        # print(targetPoses[UAV_id])
        if leave_route[UAV_id] != 0:    # 尚未回到默认路径
            # load checkpoint
            leave_route[UAV_id] -= 1
            print("UAV " + str(UAV_id) + " back to route " + str(leave_route[UAV_id]))
        else:   # 已经回到默认路径
            if no_buffer_location[UAV_id] == 1: 
                targetPoses[UAV_id].append(targetPoses[UAV_id][0])
            else:
                no_buffer_location[UAV_id] = 1
                print("del " + str(UAV_id))
        del targetPoses[UAV_id][0]
        # print("new targetPoses:",targetPoses[UAV_id][0])
        setpoint(UAV_id,targetPoses)
        return targetPoses
    else:
        # print("%d flying"%(UAV_id))
        return targetPoses


def goal_publisher(UAV_id,targetPoses): # 检测当前位置是否到达目的地，并据此调整目标
    try:
        targetPoses=CheckState(UAV_id,targetPoses,UAVpos[UAV_id][0],UAVpos[UAV_id][1])
        return targetPoses
    except:
        return False

def design_Route():
    return 0

def callback_override(msg, UAV_id):
    lastOverride[UAV_id] = isOverride[UAV_id]
    if msg.data == 0 :
        goal_pub[UAV_id] = rospy.Publisher("/xtdrone/typhoon_h480_"+str(UAV_id)+"/cmd_pose_enu", Pose, queue_size=1)
    isOverride[UAV_id] = msg.data
    if msg.data == 1 :
        goal_pub[UAV_id].unregister()
    if lastOverride[UAV_id] == 1 and isOverride[UAV_id] == 0 :
        print("UAV " + str(UAV_id) + " comming back from route" + str(leave_route[UAV_id]))
    # if lastOverride[UAV_id] == 0 and isOverride[UAV_id] == 1:
    #     add_checkpoint(UAV_id)

def add_checkpoint(UAV_id):
    global targetPoses
    targetPoses[UAV_id].insert(0,[UAVpos[UAV_id][0],UAVpos[UAV_id][1],5])
    print("UAV " + str(UAV_id) + " leaving route " + str(leave_route[UAV_id]))
    leave_route[UAV_id] += 1

# closer
def default_Route():         #每台无人机的路径
    if mode == 0:       # 六架默认
        UAV0 = [[-44.5, -0.5, 4.5], [-44.5, -42, 4.5], [113, -42, 4.5], [113, -0.5, 4.5], [35, -0.5, 4.5]]
        UAV0 = [[-44.5, -0.5, 4.5], [-44.5, -42, 4.5], [113, -42, 4.5], [113, -0.5, 4.5], [35, -0.5, 4.5]]
        UAV1 = [[35, -1.5, 4.5], [109, -1.5, 4.5], [109, -40, 4.5], [-43.5, -40, 4.5], [-43.5, -1.5, 4.5]]
        UAV2 = [[-46.5, 0, 4.5], [-46.5, 46, 4.5], [121, 46, 4.5], [121, -46, 4.5], [-46.5, -46, 4.5]]
        UAV3 = [[117, 0, 4.5], [117, 44, 4.5], [-45.5, 44, 4.5], [-45.5, -44, 4.5], [117, -44, 4.5]]
        UAV4 = [[-44.5, 0.5, 4.5], [-44.5, 42, 4.5], [113, 42, 4.5], [113, 0.5, 4.5], [35, 0.5, 4.5]]
        UAV5 = [[35, 1.5, 4.5], [109, 1.5, 4.5], [109, 40, 4.5], [-43.5, 40, 4.5], [-43.5, 1.5, 4.5]]
        # UAV0 = [[-17,-3,4.5]]
        # UAV1 = [[-14,-3,4.5]]
        # UAV2 = [[-17,0,4.5]]
        # UAV3 = [[-14,0,4.5]]
        # UAV4 = [[-17,3,4.5]]
        # UAV5 = [[-14,3,4.5]]
    
    if mode == 1:       # 六架靠外
        UAV0 = [[-44.5, -0.5, 4.5], [-44.5, -45, 4.5], [113, -45, 4.5], [113, -0.5, 4.5], [35, -0.5, 4.5]]
        UAV1 = [[35, -1.5, 4.5], [109, -1.5, 4.5], [109, -43, 4.5], [-43.5, -43, 4.5], [-43.5, -1.5, 4.5]]
        UAV2 = [[-46.5, 0, 4.5], [-46.5, 49, 4.5], [121, 49, 4.5], [121, -49, 4.5], [-46.5, -49, 4.5]]
        UAV3 = [[117, 0, 4.5], [117, 47, 4.5], [-45.5, 47, 4.5], [-45.5, -47, 4.5], [117, -47, 4.5]]
        UAV4 = [[-44.5, 0.5, 4.5], [-44.5, 45, 4.5], [113, 45, 4.5], [113, 0.5, 4.5], [35, 0.5, 4.5]]
        UAV5 = [[35, 1.5, 4.5], [109, 1.5, 4.5], [109, 43, 4.5], [-43.5, 43, 4.5], [-43.5, 1.5, 4.5]]

    if mode == 2:       # 三架默认
        UAV0=[[-45.5,-0.5,4.5],[-45,-43,4.5],[115,-43,4.5]]
        UAV1=[[3,-3,2.5],[3,-3,2.5]]
        UAV2=[[-43.5,0,4.5],[115,0,4.5],[-45,0,4.5]]
        UAV3=[[3,0,2.5],[3,0,2.5]]
        UAV4=[[-44.5,0.5,4.5],[-45,43,4.5],[115,43,4.5]]
        UAV5=[[3,3,2.5],[3,3,2.5]]
    
    return [UAV0,UAV1,UAV2,UAV3,UAV4,UAV5]

bias=[                                   #每台无人机的偏差
    [0,-3,0],[3,-3,0],
    [-3,0,0],[3,0,0],
    [0,3,0],[3,3,0]
]


if __name__ == '__main__':
    if mode == 2: 
        err_threshold = 2
        no_buffer_location = [0] * 6
    
    targetPoses = default_Route()     # 初始飞行路径
    # print("targetPoses:",targetPoses)
    rospy.init_node("typhoon_h480_search_and_find")

    for UAV_id in range(6):
        current_pose[UAV_id] = rospy.Subscriber('/typhoon_h480_'+str(UAV_id)+'/mavros/local_position/pose',PoseStamped,local_pose_callback,UAV_id,queue_size=1)#获取当前无人机位置
        goal_pub[UAV_id] = rospy.Publisher("/xtdrone/typhoon_h480_"+str(UAV_id)+"/cmd_pose_enu", Pose, queue_size=1)
        override_subscriber.append(rospy.Subscriber("/typhoon_h480_"+str(UAV_id)+"/override", Int8, callback_override, UAV_id))
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        # print("="*20)
        print("override: " + str(isOverride))
        for UAV_id in range(6):
            # print(err_threshold)
            # print(str(UAV_id) + str(targetPoses[UAV_id]))
            if isOverride[UAV_id] == 0:
                setpoint(UAV_id,targetPoses)
            if isOverride[UAV_id] == 1:
                add_checkpoint(UAV_id)
        try:
            rate.sleep()
        except:
            continue
