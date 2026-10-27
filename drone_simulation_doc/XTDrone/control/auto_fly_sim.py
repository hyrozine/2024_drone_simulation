#coding=UTF-8
import rospy
from geometry_msgs.msg import Twist,PoseStamped, Pose
import sys, select, os
import tty, termios
from std_msgs.msg import String

MAX_LINEAR = 20
MAX_ANG_VEL = 3
LINEAR_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01

cmd_vel_mask = False
ctrl_leader = False




def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def local_pose_callback(data):
    global pos_z, pos_x, pos_y
    pos_x = data.pose.position.x
    pos_y = data.pose.position.y
    pos_z = data.pose.position.z

    twist.linear.x = (target_x-pos_x)*Kp_xy
    twist.linear.y = (target_y-pos_y)*Kp_xy
    twist.linear.z = (target_z-pos_z)*Kp_z


if __name__=="__main__":

    control_use="auto" #switch to keyboard using 'keyboard'

    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = 'tyhoon_h480'
    multirotor_num = 6
    #control_type = sys.argv[3]
    #print("multirotor_num:"+multirotor_num)

    #---  start node initialize and publisher/suscriber ---
    rospy.init_node('robocup_autofly_position')
    multi_cmd_vel_flu_pub = [None]*6
    multi_cmd_pub = [None]*6
    #for i in range(6):
    local_pose_sub = rospy.Subscriber('/typhoon_h480_3/mavros/local_position/pose', PoseStamped,local_pose_callback,queue_size=1)
    multi_cmd_vel_flu_pub[3] = rospy.Publisher('/xtdrone/typhoon_h480_3/cmd_vel_flu', Twist, queue_size=1) #这个实在传
    multi_cmd_pub[3] = rospy.Publisher('/xtdrone/typhoon_h480_3/cmd',String,queue_size=3)  #这个是在传keyboard发送的指令（如offboard等）

    cmd= String()
    twist = Twist()
    local_pose=PoseStamped()
    pos_x=0
    pos_y=0
    pos_z=0
    target_x=0.0
    target_y=0.0
    target_z=10.0

    Kp_xy = 0.5
    Kp_z = 1


    while(1):
        key = getKey()
        #if key == 'w':
		
        if key == 'w':
            target_x=10.0
            target_y=0.0
            target_z=10.0
        elif key == 'd':
            target_x=10.0
            target_y=10.0
            target_z=10.0

		#elif (key=='\x03'):
			#break

        #for i in range(multirotor_num):
        multi_cmd_vel_flu_pub[3].publish(twist)  
        multi_cmd_pub[3].publish(cmd)

        cmd=''
        print("\n current target:%.2f" %(target_z))
        print("currently:\t forward vel %.2f\t leftward vel %.2f\t upward vel %.2f\t angular %.2f " % (twist.linear.x, twist.linear.y, twist.linear.y, twist.angular.z))
        print("pose x:%.2f\t pose y:%.2f\t pose z:%.2f\n"%(pos_x,pos_y,pos_z))

	
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)





    
