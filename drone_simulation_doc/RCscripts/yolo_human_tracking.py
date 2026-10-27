import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String 
import sys 
from pyquaternion import Quaternion
from darknet_ros_msgs.msg import BoundingBoxes
import math

class Pid():
    def __init__(self, exp_val, kp, ki, kd):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.now_val = 0
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.change_val = 0

    def cmd_pid(self, exp_val):
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_err = exp_val - self.now_val
        self.change_val = self.KP * (self.now_err - self.last_err) + self.KI * \
            self.now_err + self.KD * (self.now_err - 2 * self.last_err
                                      + self.last_last_err)
        self.now_val += self.change_val
        return self.now_val

x_velocity = 0
y_velocity = 0
z_velocity = 0
now_x_velocity = 0
now_y_velocity = 0
now_z_velocity = 0
real_x_velocity = 0
real_y_velocity = 0
real_z_velocity = 0

x_velocity_granu = 0.1
y_velocity_granu = 0.1
z_velocity_granu = 0.15

trace_id = -1
trace_tim_stamp = None
trace_lose_tim_thre = 0.5
def darknet_callback(data):
    global find_cnt, twist, cmd, target_height_mask, target_height,theta, get_time
    global x_velocity, y_velocity, now_x_velocity, now_y_velocity, x_velocity_granu, y_velocity_granu, real_x_velocity, real_y_velocity
    global z_velocity, now_z_velocity, real_z_velocity
    global trace_id, trace_tim_stamp, trace_lose_tim_thre
    for target in data.bounding_boxes:
        if trace_id == -1 :
            break

        if trace_tim_stamp == None :  
            break

        if target.id == 5 :
            continue

        if trace_id == target.id :
            trace_id = target.id
            trace_tim_stamp = rospy.get_time()
            # print(target.id)
            # print(theta)
            # print('find human')
            z = height / math.sin(theta)
            u = (target.xmax+target.xmin)/2
            v = (target.ymax+target.ymin)/2
            u_ = u-u_center
            v_ = v-v_center
            u_velocity = -Kp_xy*u_
            v_velocity = -Kp_xy*v_
            # y_velocity = v_velocity*z/(v_*math.cos(theta)+fy*math.sin(theta))
            # x_velocity = (z*u_velocity-u_*math.cos(theta)*y_velocity)/fx
            x_velocity = v_velocity*z/(v_*math.cos(theta)+fy*math.sin(theta))
            y_velocity = (z*u_velocity-u_*math.cos(theta)*x_velocity)/fx
            z_velocity = Kp_z*(target_height-height)
            # print("target tan theta"+str(x_velocity/y_velocity))

            if (x_velocity > 3):
                x_velocity = 3
            elif(x_velocity<-3):
                x_velocity=-3

            if (y_velocity > 3):
                y_velocity = 3
            elif(y_velocity<-3):
                y_velocity=-3

            if (z_velocity > 3):
                z_velocity = 3
            elif(z_velocity<-3):
                z_velocity=-3
            
            if (x_velocity > now_x_velocity) :
                now_x_velocity += x_velocity_granu
            elif (x_velocity < now_x_velocity) :
                now_x_velocity -= x_velocity_granu

            if (y_velocity > now_y_velocity) :
                now_y_velocity += y_velocity_granu
            elif (y_velocity < now_y_velocity) :
                now_y_velocity -= y_velocity_granu

            if (z_velocity > now_z_velocity) :
                now_z_velocity += z_velocity_granu
            elif (z_velocity < now_z_velocity) :
                now_z_velocity -= z_velocity_granu

            if (now_x_velocity < 0.2 and now_x_velocity > -0.2) :
                real_x_velocity = 0
            else :
                real_x_velocity = now_x_velocity

            if (now_y_velocity < 0.2 and now_y_velocity > -0.2) :
                real_y_velocity = 0
            else :
                real_y_velocity = now_y_velocity

            if ((target_height - height) < 0.2 and (target_height - height) > -0.2) :
                real_z_velocity = 0
            else :
                real_z_velocity = now_z_velocity

            # print("x_v = " + str(x_velocity) + " and y_v = " + str(y_velocity))
            # print("now_x_v = " + str(now_x_velocity) + "and now_y_v = " + str(now_y_velocity))
            # print("real_x_v = " + str(real_x_velocity) + "and real_y_v = " + str(real_y_velocity))
            twist.linear.x = real_x_velocity
            twist.linear.y = real_y_velocity
            # twist.linear.y = y_velocity
            twist.linear.z = real_z_velocity
            print(height)
            # twist.linear.z = 0
            cmd = ''
            find_cnt = find_cnt + 1
            get_time = False
            return

    for target in data.bounding_boxes:
        # print(target.id)
        # print(theta)
        if(target.id < 5):
            if trace_tim_stamp != None:
                if rospy.get_time() - trace_tim_stamp > trace_lose_tim_thre:
                    trace_id = target.id
                    trace_tim_stamp = rospy.get_time()
            elif trace_tim_stamp == None:
                trace_id = target.id
                trace_tim_stamp = rospy.get_time()
            return
    
        
def local_pose_callback(data):
    global height, target_height, target_set
    height = data.pose.position.z    
    if not target_set:
        target_height = height     
        target_set = True    

def cam_pose_callback(data):
    global theta
    q = Quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z)
    theta = q.yaw_pitch_roll[1]
    # print(data)
    # print("own tan theta:"+str(math.tan(q.yaw_pitch_roll[0])))
            
if __name__ == "__main__":
    height = 0  
    target_height = 4
    target_set = True
    find_cnt = 0
    find_cnt_last = 0
    not_find_time = 0
    get_time = False
    twist = Twist()
    cmd = String()
    theta = 0
    # u_center=640/2
    # v_center=360/2
    # u_center = 252
    # v_center = 73
    u_center = 260
    v_center = 170

    fx = 205.46963709898583
    fy = 205.46963709898583
    Kp_xy = 0.6
    Kp_z = 0.6
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    rospy.init_node('yolo_human_tracking_{}'.format(vehicle_id))
    rospy.Subscriber("/uav_"+vehicle_id+"/darknet_ros/bounding_boxes", BoundingBoxes, darknet_callback,queue_size=1)
    rospy.Subscriber(vehicle_type+'_'+vehicle_id+"/mavros/local_position/pose", PoseStamped, local_pose_callback,queue_size=1)
    rospy.Subscriber('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cam_pose', PoseStamped, cam_pose_callback,queue_size=1)
    cmd_vel_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd_vel_flu', Twist, queue_size=1)
    cmd_pub = rospy.Publisher('/xtdrone/'+vehicle_type+'_'+vehicle_id+'/cmd', String, queue_size=1)
    rate = rospy.Rate(30) 
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        cmd_pub.publish(cmd)
        if find_cnt - find_cnt_last == 0:
            if not get_time:
                not_find_time = rospy.get_time()
                get_time = True
            if rospy.get_time() - not_find_time > 2.0:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                cmd = 'HOVER'
                print(cmd)
                get_time = False
        find_cnt_last = find_cnt
        rate.sleep()