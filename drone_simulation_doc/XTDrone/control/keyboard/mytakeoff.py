import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios
from std_msgs.msg import String


MAX_LINEAR = 20
MAX_ANG_VEL = 3
LINEAR_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.02

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

def print_msg():
    if ctrl_leader:
        print(msg2leader)
    else:
        print(msg2all)      

if __name__=="__main__":

    settings = termios.tcgetattr(sys.stdin)

    multirotor_type = sys.argv[1]
    multirotor_num = int(sys.argv[2])
    control_type = sys.argv[3]
    
    if multirotor_num == 18:
        formation_configs = ['waiting', 'cuboid', 'sphere', 'diamond']
    elif multirotor_num == 9:
        formation_configs = ['waiting', 'cube', 'pyramid', 'triangle']
    elif multirotor_num == 6:
        formation_configs = ['waiting', 'T', 'diamond', 'triangle']
    elif multirotor_num == 1:
        formation_configs = ['stop controlling']
    
    cmd= String()
    twist = Twist()   
    
    rospy.init_node(multirotor_type + '_multirotor_keyboard_control')
    
    if control_type == 'vel':
        multi_cmd_vel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_vel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_vel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=3)
        leader_cmd_vel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_vel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=1)
 
    else:
        multi_cmd_accel_flu_pub = [None]*multirotor_num
        multi_cmd_pub = [None]*multirotor_num
        for i in range(multirotor_num):
            multi_cmd_accel_flu_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd_accel_flu', Twist, queue_size=1)
            multi_cmd_pub[i] = rospy.Publisher('/xtdrone/'+multirotor_type+'_'+str(i)+'/cmd',String,queue_size=1)
        leader_cmd_accel_flu_pub = rospy.Publisher("/xtdrone/leader/cmd_accel_flu", Twist, queue_size=1)
        leader_cmd_pub = rospy.Publisher("/xtdrone/leader/cmd", String, queue_size=3)

    forward  = 0.0
    leftward  = 0.0
    upward  = 4.0
    angular = 0.0

    i=0
    while(1):

        if(i==0):
            cmd='OFFBOARD'
        else:
            cmd='ARM'
        i=i+1
        if forward > MAX_LINEAR:
            forward = MAX_LINEAR
        elif forward < -MAX_LINEAR:
            forward = -MAX_LINEAR
        if leftward > MAX_LINEAR:
            leftward = MAX_LINEAR
        elif leftward < -MAX_LINEAR:
            leftward = -MAX_LINEAR
        if upward > MAX_LINEAR:
            upward = MAX_LINEAR
        elif upward < -MAX_LINEAR:
            upward = -MAX_LINEAR
        if angular > MAX_ANG_VEL:
            angular = MAX_ANG_VEL
        elif angular < -MAX_ANG_VEL:
            angular = - MAX_ANG_VEL
            
        twist.linear.x = forward; twist.linear.y = leftward ; twist.linear.z = upward
        twist.angular.x = 0.0; twist.angular.y = 0.0;  twist.angular.z = angular

        for i in range(multirotor_num):
            if ctrl_leader:
                if control_type == 'vel':
                    leader_cmd_vel_flu_pub.publish(twist)
                else:
                    leader_cmd_accel_flu_pub.publish(twist)
                leader_cmd_pub.publish(cmd)
            else:
                if not cmd_vel_mask:
                    if control_type == 'vel':
                        multi_cmd_vel_flu_pub[i].publish(twist)   
                    else:
                        multi_cmd_accel_flu_pub[i].publish(twist)
                multi_cmd_pub[i].publish(cmd)
                
        cmd = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

