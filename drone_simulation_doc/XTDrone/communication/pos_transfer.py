import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import Int8
from pyquaternion import Quaternion
import sys
import math
import numpy as np
rospy.init_node("pos_transfer")
rate = rospy.Rate(30)
typhoon_h480_bias= np.array( [
    [0,-3,0],[3,-3,0],
    [0,0,0],[3,0,0],
    [0,3,0],[3,3,0]
] )
bias = typhoon_h480_bias
class Communication:
    def __init__(self):
        
        self.multirotor_num = 6
        self.multirotor_type ="typhoon_h480"
        self.multi_cmd_pose_enu_pub = [None]*self.multirotor_num
        self.multi_cmd_pose_enu_sub = [None]*self.multirotor_num
        self.multi_track_state_sub = [None]*self.multirotor_num
        self.multi_track_state = [0]*self.multirotor_num
        for i in range(self.multirotor_num):
            self.multi_cmd_pose_enu_pub[i] = rospy.Publisher('/xtdrone/'+self.multirotor_type+'_'+str(i)+'/cmd_pose_enu', Pose, queue_size=1)
            self.multi_cmd_pose_enu_sub[i] = rospy.Subscriber(self.multirotor_type+'_'+str(i)+'/cmd_pose_enu',PoseStamped,self.cmd_pose_enu_callback,i,queue_size=3)
            self.multi_track_state_sub[i] = rospy.Subscriber(self.multirotor_type+'_'+str(i)+'/override',Int8,self.track_state_callback,i,queue_size=1)
    def track_state_callback(self ,msg , i):
        self.multi_track_state[i]=msg.data
    def cmd_pose_enu_callback(self, msg  , i ):
        local_cmd = msg.pose
        if msg.header.frame_id=='ego' and  self.multi_track_state[i] != 2:
            local_cmd.position.x = msg.pose.position.x-bias[i,0]
            local_cmd.position.y = msg.pose.position.y-bias[i,1]
            local_cmd.position.z = msg.pose.position.z-bias[i,2]
            self.multi_cmd_pose_enu_pub[i].publish(local_cmd)
        
    def start(self):
        rospy.spin()
        
if __name__ == '__main__':
    communication = Communication()
    communication.start()