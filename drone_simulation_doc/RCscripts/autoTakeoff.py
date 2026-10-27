# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
import sys
import math
import numpy as np
rospy.init_node("autotakeoff")
rate = rospy.Rate(30)
typhoon_h480_pose= np.array( [
    [0,0,5],[0,0,5],
    [0,0,5],[0,0,5],
    [0,0,5],[0,0,5]
] )
pos = typhoon_h480_pose
class Takeoff:
    def __init__(self) :
        # self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.multirotor_num = 6
        self.takeoffed=0
        self.multirotor_type ="typhoon_h480"
        self.mavros_state_sub = [None]*self.multirotor_num
        self.mavros_state = [State()]*self.multirotor_num
        self.multi_cmd_pose_enu_pub = [None]*self.multirotor_num
        self.multi_cmd_pub = [None]*self.multirotor_num
        self.multi_timer = [None]*self.multirotor_num
        for i in range(self.multirotor_num):
            self.multi_cmd_pose_enu_pub[i] = rospy.Publisher('/xtdrone/'+self.multirotor_type+'_'+str(i)+'/cmd_pose_enu', Pose, queue_size=1)
            self.multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+self.multirotor_type+'_'+str(i)+'/cmd',String,queue_size=3)
            self.mavros_state_sub[i] = rospy.Subscriber(self.multirotor_type+'_'+str(i)+'/mavros/state',State,self.mavros_state_cb,i,queue_size=3)
    def mavros_state_cb(self,msg ,i):
        self.mavros_state[i] =msg
    def takedoff_cb(self,event):
        self.takeoffed+=1
    def start(self):
        while not rospy.is_shutdown() and self.takeoffed<6:
            for i in range(self.multirotor_num):
                self.fsm_run(i)
            # self.target_motion_pub.publish(self.target_motion)
            rate.sleep()
    def fsm_run(self,i):     
        if self.mavros_state[i].mode != "OFFBOARD":
            self.multi_cmd_pub[i].publish("OFFBOARD")
        else:
            if self.mavros_state[i].armed :
                pose = Pose()
                pose.position.x = pos[i][0]
                pose.position.y = pos[i][1]
                pose.position.z = pos[i][2]
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1
                self.multi_cmd_pose_enu_pub[i].publish(pose)
                self.multi_timer[i] =rospy.Timer(rospy.Duration(0.2), self.takedoff_cb,oneshot=True)
            else:
                self.multi_cmd_pub[i].publish("ARM")
        

if __name__ == '__main__':
    mission = Takeoff()
    mission.start()
 
