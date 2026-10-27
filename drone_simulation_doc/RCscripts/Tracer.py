#!/usr/bin/env python3
import math
import numpy as np
import rospy 
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8, String
import message_filters as mf
import threading
from gazebo_msgs.srv import DeleteModel,GetModelState
from queue import Queue

#------------------------------几个msg的Compact Message Definition------------------------------
#----------------------------------------
# 1. PoseStamped
# std_msgs/Header header
# geometry_msgs/Pose pose
#----------------------------------------
# 2. Odometry
# std_msgs/Header header
# string child_frame_id
# geometry_msgs/PoseWithCovariance  pose
# geometry_msgs/TwistWithCovariance twist



from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo

# 状态机
# 0 -> 默认轨迹
# 1 -> 追踪


# 常数
# u_max = 640             # 横向最大像素
# v_max = 480             # 纵向最大像素
u_max = 640             # 横向最大像素
v_max = 480             # 纵向最大像素
u_fov = 1.047198        # 横向最大视角

center_height = 0.9     # 人中心高度
trace_height = 2        # 追踪高度
vel_trace_thre = 15     # 速度追踪阈值

time_lim = 5
time_counter = 0

time_bb = 0
time_bb_status = 0

# depth_fix = -2.2
depth_fix = 0

judge_pub_tim_thre = 0.5

# err_cor[2] = []
init_pos = np.array([   # 小飞机初始坐标
    [0, -3, 0],
    [3, -3, 0],
    [0, 0, 0],
    [3, 0, 0],
    [0, 3, 0],
    [3, 3, 0]
])

#----------------------------------------
# 6个对象的数据共享区，每个无人机都知道那个对象在被那个无人机追踪
# 这样不用再简历通信，非常巧妙
# onTracing[0]          追踪目标的位置数据(世界坐标系)
onTracing = {           # 记录actor被谁追踪着
        'actor_red1' : None,
        'actor_red2' : None,
        'actor_blue' : None, 
        'actor_white' : None,
        'actor_brown' : None, 
        'actor_green' : None
}

bLeft = {               # actor 丢失状态
        'actor_red1' : True,
        'actor_red2' : True,
        'actor_blue' : True, 
        'actor_white' : True,
        'actor_brown' : True, 
        'actor_green' : True
}


class_tag_cls = {       # 用于向裁判系统发布话题
    'actor_red1' : ('red',rospy.Publisher("/actor_red1_info",ActorInfo,queue_size=1)),
    'actor_red2' : ('red',rospy.Publisher("/actor_red2_info",ActorInfo,queue_size=1)),
    'actor_blue' : ('blue',rospy.Publisher("/actor_blue_info",ActorInfo,queue_size=1)), 
    'actor_white' : ('white',rospy.Publisher("/actor_white_info",ActorInfo,queue_size=1)),
    'actor_brown' : ('brown',rospy.Publisher("/actor_brown_info",ActorInfo,queue_size=1)), 
    'actor_green' : ('green',rospy.Publisher("/actor_green_info",ActorInfo,queue_size=1))
}


fpx = u_max / 2 / math.asin(u_fov/2)    # 像素焦距

trace_thre = 4                          # 跟随距离
# trace_k = 0

#----------------------------------------
# 坐标系转换
# @brief            把图片坐标系转换成旋转矩阵的形式
Muv2xy = np.array(
    [
        [1, 0, -u_max/2],
        [0, 1, -v_max/2],
        [0, 0,     1]
    ])

# xy转XcYcZc
# @brief            图片坐标系转换成相机坐标系
# @param vec_xy1    图片坐标系的(x,y)坐标
# @param rho        目标点距离相机的距离
def Matrix_xy2XcYcZc(vec_xy1, rho):
    fac_x2Xc = rho / math.sqrt(fpx ** 2 + vec_xy1[0][0] ** 2 + vec_xy1[1][0]**2)
    M = np.array([
        [fac_x2Xc,  0,          0],
        [0,         fac_x2Xc,   0],
        [0,         0,          fpx * fac_x2Xc],
        [0,         0,          1]
    ])
    return M

# XcYcZc转XwYwZw
# @brief                相机坐标系转换为世界坐标系
# @param pos            相机坐标系下的位置信息
# @param orth           相机坐标系下的姿态信息
def Matrix_XcYcZc2XwYwZW(pos, orth):
    x, y, z, w = orth
    R = np.array([  [1-2*y**2-2*z**2, 2*x*y-2*w*z, 2*x*z+2*w*y],
                    [2*x*y+2*w*z    , 1-2*x**2-2*z**2,    2*y*z-2*w*x],
                    [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x**2-2*y**2]])
    
    R = np.array([[-R[0][1], -R[0][2], R[0][0]],
                  [-R[1][1], -R[1][2], R[1][0]],
                  [-R[2][1], -R[2][2], R[2][0]]])
    return np.array([
        [R[0][0], R[0][1], R[0][2], pos[0]],
        [R[1][0], R[1][1], R[1][2], pos[1]],
        [R[2][0], R[2][1], R[2][2], pos[2]],
        [0, 0, 0,    1]
    ])
    # return np.linalg.inv(np.array([
    #     [1-2*q2**2-2*q3**2, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2,    pos[0]],
    #     [2*q1*q2+2*q0+q3, 1-2*q1**2-2*q3**2, 2*q2*q3-2*q0*q1,    pos[1]],
    #     [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*q1**2-2*q2**2,    pos[2]],
    #     [0, 0, 0,    1]
    # ]))

# 获取目标的坐标
# @param u              光心的横坐标(图像坐标系 - 可以理解为图像坐标系中心的x坐标)
# @param v              光心的纵坐标(图像坐标系 - 可以理解为图像坐标系中心的y坐标)
# @param rho            目标点距离相机的距离
# @param pos            相机的位置信息
# @param orth           相机的姿态信息
def GetTargetPosition(u, v, rho, pos, orth):
    if math.isnan(rho):
        rho_cal = 1
    else:
        rho_cal = rho + depth_fix
    # print("depth = \t" + str(rho_cal), end="")
    # print("(u, v) = " + str([u, v]))
    vec_uv = np.array([[u], [v], [1]])
    vec_xy = Muv2xy.dot(vec_uv)
    # 把图片坐标系的坐标数据转换为此相机坐标系的坐标数据
    vec_c = Matrix_xy2XcYcZc(vec_xy, rho_cal).dot(vec_xy)           # .dot() 实现的是矩阵的乘法
    # vec_c_T = np.array([vec_c[2], -vec_c[0], -vec_c[1], [1]])
    # 把相机坐标系的坐标数据转换为世界坐标系的坐标数据
    vec_w = Matrix_XcYcZc2XwYwZW(pos, orth).dot(vec_c)

    # 获取一个距离数据?
    if math.isnan(rho):
        rho_fix = (pos[2] - center_height) / (pos[2] - vec_w[2][0]) * rho_cal
        return GetTargetPosition(u, v, rho_fix, pos, orth)
    else:
        return vec_w[0:3].T[0]


# 更新成员是否丢失的数据
# 方法find() 如果没有找到
def UpdateCapture(msg):
    
    if msg.data.find("0") == -1:
        bLeft['actor_green'] = False
    if msg.data.find("1") == -1:
        bLeft['actor_blue'] = False
    if msg.data.find("2") == -1:
        bLeft['actor_brown'] = False
    if msg.data.find("3") == -1:
        bLeft['actor_white'] = False
    if msg.data.find("4") == -1:
        bLeft['actor_red1'] = False
    if msg.data.find("5") == -1:
        bLeft['actor_red2'] = False


# PID类
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

    # 增量式PID
    def cmd_pid(self, exp_val):
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_err = exp_val - self.now_val
        self.change_val = self.KP * (self.now_err - self.last_err) + self.KI * \
            self.now_err + self.KD * (self.now_err - 2 * self.last_err
                                      + self.last_last_err)
        self.now_val += self.change_val
        return self.now_val

class Tracer:
    # index 为 typhoon 的编号
    def __init__(self, index) -> None:
        self.index = index
        self.judge_pub_tim_stamp = 0
        self.judge_pub_bb_cnt = 0
        self.judge_pub_x = 0.0
        self.judge_pub_y = 0.0
        self.last_trace_pos = None
        self.slope = 0
        self.row = 2
        self.colomus = 10
        self.bb_data = np.array([0.0, 0.0])
        self.vec_x = 0.0
        self.vec_y = 0.0
        # print("typhoon_h480_{}".format(index))

        '''
        #------------------------------Tracer订阅消息配置------------------------------
        # 订阅下相机的坐标信息
        # pos_sub = mf.Subscriber("/xtdrone/typhoon_h480_{}/cam_pose".format(index), PoseStamped) # orth_sub = mf.Subscriber("/xtdrone/typhoon_h480_{}/cam_pose".format(index), PoseStamped)
        pos_sub = mf.Subscriber("/xtdrone/typhoon_h480_{}/cam_pose".format(index), PoseStamped)
        print("/xtdrone/typhoon_h480_{}/cam_pose topic has been subscribed".format(index))
        # bounding_box 话题的订阅
        # yolo_sub = mf.Subscriber("/typhoon_h480_{}/darknet_ros/bounding_boxes".format(index), BoundingBoxes)
        yolo_sub = mf.Subscriber("/uav_{}/darknet_ros/bounding_boxes".format(index), BoundingBoxes)
        print("/uav_{}/darknet_ros/bounding_boxes topic has benn subscribed".format(index))
        # 相机图片 话题的订阅 - 订阅的是深度相机的信息
        disp_sub = mf.Subscriber("/typhoon_h480_{}/realsense/depth_camera/depth/image_raw".format(index), Image)
        print("/typhoon_h480_{}/realsense/depth_camera/depth/image_raw topic has been subscribed".format(index))
        '''

        
        # 同步多个订阅者
        # @param [pos_sub, yolo_sub, disp_sub]      需要同步的话题
        # @param 10                                 最大允许的时间偏差
        # @param 1                                  最小需要同步的消息数量
        # @func  registerCallback                   注册回调函数
        # pos_yolo_disp_sync = mf.ApproximateTimeSynchronizer([pos_sub, disp_sub, yolo_sub], queue_size=10, slop=1, allow_headerless=True)
        # pos_yolo_disp_sync.registerCallback(self.Callback_Pos_Yolo_Disp)
        
        #------------------------------Tracer发布消息配置------------------------------
        # 6个无人机 ego 追踪目标的话题
        self.ego_pub = rospy.Publisher('/typhoon_h480_{}/move_base_simple/goal'.format(index), PoseStamped, queue_size=1)
        # 6个无人机状态机的话题
        self.override_pub = rospy.Publisher('/typhoon_h480_{}/override'.format(index), Int8, queue_size=1)
        # 6个无人机目标里程计的话题                     # TODO: 需要进一步理解
        self.tracer_pub = rospy.Publisher('/typhoon_h480_{}_tracker/target/odom'.format(index), Odometry, queue_size=1)
        # 6个无人机关闭ego的话题
        self.cutoff_ego_pub = rospy.Publisher('/typhoon_h480_{}_tracker/triger'.format(index), PoseStamped, queue_size=1)
        
        # self.cmd_vel_pub = rospy.Publisher('/xtdrone/typhoon_h480_{}/cmd_vel_flu'.format(index), Twist, queue_size=1)
        # self.cmd_pub = rospy.Publisher('/xtdrone/typhoon_h480_{}/cmd'.format(index), String, queue_size=1)

        self.vec_uv = np.array([[0], [0], [1]])
        # self.v = 0
        # self.u = 0
        self.trace_loc = None                       # 追踪目标点坐标(这个 loc 应该是location的缩写)
        self.override = 0                           # 追踪状态机
        
        self.last_trace = rospy.get_time() - 10     # 记录当前的时间戳          # TODO: 需要进一步理解
        self.bounding_boxes = []
        
        self.last_target_class = None               # 上一个目标的类型
        self.trace_object_class = None              # 追踪的目标的类型
        # self.hovered = False
    
    # 创建里程计消息
    def CreateOdomMsg(self, trace_loc):
        odom_msg = Odometry()                           # 初始化里程计消息
        odom_msg.header.frame_id = 'map'                # 该里程计相对于map坐标系
        
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z = trace_loc
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = (0,0,0,1)

        odom_msg.header.stamp = rospy.Time.now()        # 时间戳
        return odom_msg
        
    # 计算本机坐标(要替换为imu的值)
    def Callback_GetPos(self, msg):
        pos = msg.pose.position
        self.pos = np.array([pos.x, pos.y, pos.z])      # 将位置信息转换为 np.array() 类型的数据
        
        orth = msg.pose.orientation                     # 将姿态信息转换为 np.array() 类型的数据
        self.orth = np.array([orth.x, orth.y, orth.z, orth.w])
        # print(self.orth)
        
        # print("pos:{}\torth:{}".format(self.pos, self.orth))
        

    def Callback_vec(self, msg):
        self.vec_x = msg.linear.x
        self.vec_y = msg.linear.y
        print("vec_x = " + str(self.vec_x) + "vec_y = " + str(self.vec_y))

    # bounding_boxes的回调函数
    def Callback_Yolo(self, msg):           
        # 等待相关的消息过来
        sub_get_depth = rospy.Subscriber("/typhoon_h480_{}/realsense/depth_camera/depth/image_raw".format(self.index), Image, callback=self.Callback_disp, queue_size=1)
        rospy.wait_for_message("/typhoon_h480_{}/realsense/depth_camera/depth/image_raw".format(self.index), Image)
        sub_get_depth.unregister()
        sub_get_pose = rospy.Subscriber("/xtdrone/typhoon_h480_{}/cam_pose".format(self.index), PoseStamped, callback=self.Callback_GetPos, queue_size=1)
        rospy.wait_for_message("/xtdrone/typhoon_h480_{}/cam_pose".format(self.index), PoseStamped)
        sub_get_pose.unregister()
        sub_get_vec = rospy.Subscriber('/xtdrone/typhoon_h480'+'_'+ str(self.index) +'/cmd_vel_flu', Twist, callback=self.Callback_vec, queue_size=1)
        rospy.wait_for_message('/xtdrone/typhoon_h480'+'_'+ str(self.index) +'/cmd_vel_flu', Twist)
        sub_get_vec.unregister()

        global time_bb_status, judge_pub_tim_thre

        if self.judge_pub_tim_stamp == 0:
            self.judge_pub_tim_stamp = rospy.get_time()
            return
        time_bb_status = 1
        self.bounding_boxes = msg.bounding_boxes
        # print(self.bounding_boxes)
        self.SelectTrace()
        if self.override == 1:
            
            if self.trace_object_pos is not None:
                # self.bb_data = np.insert(self.bb_data, 0, np.array([self.trace_object_pos[0], self.trace_object_pos[1]]))
                # print(self.bb_data)
                # if (self.bb_data.shape[0] >= self.row * self.colomus):
                #     print(self.bb_data.shape)
                #     self.bb_data = self.bb_data[: self.row * self.colomus].reshape(self.colomus, self.row)
                #     X = self.bb_data[:, 0]
                #     y = self.bb_data[:, 1]
                #     m = np.cov(X, y)[0, 1]/np.var(X)
                #     theta_m = math.atan(m)

                #     print(m)

                # self.tracer_pub.publish(self.CreateOdomMsg(self.trace_object_pos))
                # 向裁判系统发布追踪到的数据信息
                # 至少每次间隔0.5s向裁判系统发布一次消息
                if (rospy.get_time() - self.judge_pub_tim_stamp > judge_pub_tim_thre):
                    if (self.vec_y < -1):
                        self.trace_object_pos[1] -= 1.5
                    if (self.vec_y > 0.9):
                        self.trace_object_pos[0] -= 1.5
                    if (self.vec_x <= 0.2 and self.vec_x >= -0.2):
                        self.trace_object_pos[0] -= 1.2
                    
                    self.judge_pub_tim_stamp = rospy.get_time()
                    self.judge_pub(self.trace_object_class, self.trace_object_pos)

    
    # 三个同步订阅话题的回调函数
    # 三个订阅分别为:
    # 1.    相机位置坐标话题
    # 2.    bounding_boxes的话题
    # 3.    6架无人机的image_raw的话题
    def Callback_Pos_Yolo_Disp(self, pos, disp, yolo):
        # print("pos:{}|yolo:{}".format(pos.header.stamp, yolo.header.stamp))
        # print("1234")
        self.Callback_GetPos(pos)
        self.Callback_disp(disp)
        self.Callback_Yolo(yolo)
    
    # 换一种思路进行话题的同步
    def sync_process(self):
        rate = rospy.Rate(10)
        # rospy.Subscriber("/xtdrone/typhoon_h480_{}/cam_pose".format(self.index), PoseStamped, callback=self.Callback_Blank, queue_size=1)
        while not rospy.is_shutdown():
            sub_get_bb = rospy.Subscriber("/uav_{}/darknet_ros/bounding_boxes".format(self.index), BoundingBoxes, callback=self.Callback_Yolo, queue_size=1)
            rospy.wait_for_message("/uav_{}/darknet_ros/bounding_boxes".format(self.index), BoundingBoxes)
            sub_get_bb.unregister()
            rate.sleep()
        rospy.spin()
        
    
    # 选择追踪的目标
    def SelectTrace(self):
        
        self.trace_object_pos = None
        self.trace_object_class = ""
        min_dist = 1e9                          # 死区
        self.trace_loc = None
        # 没有目标直接返回
        if self.bounding_boxes is None or len(self.bounding_boxes) == 0:
            self.trace_object_pos = None
            return 
        
        # 重置 Tracing 的状态机
        for key in onTracing.keys():
            if onTracing[key] is not None and onTracing[key][1] == self.index:
                # print("clear:{}".format(key))
                onTracing[key] = None     
        
        
        # 遍历搜索边界盒，查看上一时刻目标是否存在，尝试追踪上一个目标
        for data in self.bounding_boxes:
            if self.trace_object_class == "" or self.trace_object_class is None:
                break
            if data.Class == "walker":
                continue
            # u = data.xmin + (data.xmax - data.xmin) / 3           # u 为边框中心的x坐标(大概)
            v = data.ymin + (data.ymax - data.ymin)/8          # v 为边框中心的y坐标(大概)
            u = (data.xmin + data.xmax) / 2
            # print('u='+str(u))
            # 获取坐标
            # @param u                              检测框中心点x坐标
            # @param v                              检测框中心点y坐标
            # @param sels.disp                      原生图片的深度数据
            # @param self.pos                       相机位置信息
            # @param self.orth                      相机姿态信息
            # print(self.disp[int(v)][int(u)])
            depth_disp = (self.disp[int(v)][int(u)] + self.disp[int(v - 1)][int(u)] + self.disp[int(v)][int(u + 1)] + self.disp[int(v + 1)][int(u)] + self.disp[int(v)][int(u - 1)] \
                            + self.disp[int(v - 1)][int(u - 1)] + self.disp[int(v + 1)][int(u - 1)] + self.disp[int(v - 1)][int(u + 1)] + self.disp[int(v + 1)][int(u + 1)])  / 9
            # target_pos = GetTargetPosition(u, v, self.disp[int(v)][int(u)], self.pos, self.orth)
            target_pos = GetTargetPosition(u, v, depth_disp, self.pos, self.orth)
            # if(u<190):
            #     target_pos[0]+=0.6
            #     target_pos[1]-=0.9
            # elif(u>=190 and u<250):
            #     target_pos[1]-=0.4
            # elif(u>250 and u<360):
            #     target_pos[1]+=0.8
            # elif(u>390 and u<500):
            #     target_pos[0]-=1.0
            #     target_pos[1]+=1.5
            # elif(u>=500):
            #     target_pos[0]-=1.6
            #     target_pos[1]+=1.9
            # print("target pose : ", target_pos)

            # 如果不是两个颜色相同的小红人的化，这样去处理
            if self.trace_object_class != "actor_red1" and self.trace_object_class != "actor_red2":
                # data 的数据类型是 bounding_box
                # 去尝试追踪上一个目标
                if data.Class == self.trace_object_class and bLeft[data.Class] == True:
                    self.trace_object_pos = target_pos
                    # 函数 np.linalg.norm 在这里计算的是向量的欧几里得范数 - 可以理解为向量的模长
                    # 将追踪的目标控制在 trace_thre 以内, 如果距离大于 trace_thre 的话，那么就停止追踪
                    self.trace_loc = self.pos + (self.trace_object_pos - self.pos) * (1 - trace_thre / np.linalg.norm(self.trace_object_pos - self.pos))
                    # 规定追踪器的高度 - 上面相当于规定了追踪器xy平面位置
                    self.trace_loc[2] = trace_height
                    onTracing[self.trace_object_class] = (target_pos, self.index)

                    return
            # 两个小红人单独做处理
            elif np.linalg.norm(self.trace_object_pos - target_pos) < 3 and (bLeft["actor_red1"] == True or bLeft["actor_red2"] == True):
                self.trace_object_pos = target_pos
                self.trace_loc = self.pos + (self.trace_object_pos - self.pos) * (1 - trace_thre / np.linalg.norm(self.trace_object_pos - self.pos))
                self.trace_loc[2] = trace_height
                onTracing[self.trace_object_class] = (target_pos, self.index)

                return
        
        self.trace_object_class = ""
        self.trace_object_pos = None
        
        # 遍历搜索边界盒，寻找未搜索的目标
        # 如果边界盒中没有上一次追踪的目标，就做下面的处理 
        for data in self.bounding_boxes:
            # print(data)
            # 拿到bounding_box的中心点坐标
            # u = data.xmin + (data.xmax - data.xmin) / 3           # u 为边框中心的x坐标(大概)
            v = data.ymin + (data.ymax - data.ymin)  / 8        # v 为边框中心的y坐标(大概)
            u = (data.xmin + data.xmax)/2
            # v = (data.ymin + data.ymax)/2
            # 获取目标追踪点所在世界坐标系下的坐标
            # depth_disp = (self.disp[int(v)][int(u)] + self.disp[int(v - 1)][int(u)] + self.disp[int(v)][int(u + 1)] + self.disp[int(v + 1)][int(u)] + self.disp[int(v)][int(u - 1)]) / 5
            depth_disp = (self.disp[int(v)][int(u)] + self.disp[int(v - 1)][int(u)] + self.disp[int(v)][int(u + 1)] + self.disp[int(v + 1)][int(u)] + self.disp[int(v)][int(u - 1)] \
                            + self.disp[int(v - 1)][int(u - 1)] + self.disp[int(v + 1)][int(u - 1)] + self.disp[int(v - 1)][int(u + 1)] + self.disp[int(v + 1)][int(u + 1)])  / 9
            # target_pos = GetTargetPosition(u, v, self.disp[int(v)][int(u)], self.pos, self.orth)
            target_pos = GetTargetPosition(u, v, depth_disp, self.pos, self.orth)
            if data.Class == "walker":
                continue
            elif data.Class == "actor_red":
                if onTracing["actor_red1"] is not None and np.linalg.norm(onTracing["actor_red1"][0] - target_pos) < 4.0:
                    # actor_red1 已跟踪
                    continue
                if onTracing["actor_red2"] is not None and np.linalg.norm(onTracing["actor_red2"][0] - target_pos) < 4.0:
                    # actor_red2 已跟踪
                    continue
                # 如果检测到是小红人的话，那么先追踪小红人1
                elif onTracing["actor_red1"] is None and bLeft["actor_red1"] == True:
                    if np.linalg.norm(target_pos - self.pos) < min_dist:
                        self.trace_object_class = "actor_red1"
                        self.trace_object_pos = target_pos                      
                        min_dist = np.linalg.norm(target_pos - self.pos)
                        
                elif bLeft["actor_red2"] == True:
                    if np.linalg.norm(target_pos - self.pos) < min_dist:
                        self.trace_object_class = "actor_red2"
                        self.trace_object_pos = target_pos
                        min_dist = np.linalg.norm(target_pos - self.pos)
                        
            # 追踪除了小红人以外的其他人
            elif onTracing[data.Class] is None and bLeft[data.Class] == True:
                if np.linalg.norm(target_pos - self.pos) < min_dist:
                    self.trace_object_class = data.Class
                    self.trace_object_pos = target_pos
                    min_dist = np.linalg.norm(target_pos - self.pos)
        
        # 清空边界盒列表
        self.bounding_boxes = []
        
        # 计算追踪点
        # 和上述尝试追踪上一个目标的思路一样
        if self.trace_object_pos is None:
            self.trace_loc = None
        else:
            # self.trace_loc = self.trace_object_pos
            self.trace_loc = self.pos + (self.trace_object_pos - self.pos) * (1 - trace_thre / np.linalg.norm(self.trace_object_pos - self.pos))
            # self.trace_loc = self.pos + self.trace_loc * (1 - trace_thre / np.linalg.norm(self.trace_loc))
            self.trace_loc[2] = trace_height
            # self.trace_loc = (self.trace_loc, (1 - trace_thre / np.linalg.norm(self.trace_loc)) > 0)
            # print(self.trace_loc)
            onTracing[self.trace_object_class] = (self.trace_object_pos, self.index)
            

    def Callback_disp(self, msg):
        # 把图片数据(准确来说是带有深度的图片数据，拿回来)
        self.disp = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)    
        # print("disp:{}".format(self.disp))
        
    def pub(self):
        global time_bb_status
        # 选择目标
        # if self.override != 2:
        #     self.SelectTrace()
        if self.trace_loc is not None:
            # print("[{}]".format(rospy.get_time()))
            if time_bb_status == 1 :
                self.last_trace = rospy.get_time()              # 记录开始追踪的时间戳
                time_bb_status = 0
            elif time_bb_status == 0 and rospy.get_time() - self.last_trace > 1.0 :
                self.trace_loc = None
            # 获取追踪的目标类型
            trace_loc_class = self.trace_object_class[:]
            if trace_loc_class is not None:
                # print(trace_loc_class)
                # 获取追踪的目标坐标
                trace_loc_local = self.trace_object_pos[:]

            if trace_loc_local is not None:
                # 选择发布检测类型的数据
                # 发布目标的类型信息
                # 发布目标的x坐标
                # 发布目标的y坐标
                # 连续15s广播恐怖分子的ID号和位置信息
                self.override = 1
                # self.judge_pub(trace_loc_class, trace_loc_local)
                # actors_pos_tmp = get_model_state('actor_' + str(i), 'ground_plane').pose.position
                # print("trace_loc " + str(trace_loc_local))
        elif rospy.get_time() - self.last_trace > 2.0:
            # 无目标转跑点  
            self.override = 0

        # 发布追踪状态机
        msg = Int8()
        msg.data = self.override
        self.override_pub.publish(msg)
        # print("<typhoon_h480_{}> Override:{}".format(self.index, self.override)) 
        
    # 用于向裁判发送话题
    def judge_pub(self, class_tag, pos):
        # print("pub:[{}] {} ".format(class_tag, pos))
        msg = ActorInfo()
        cls, publisher = class_tag_cls[class_tag]
        msg.cls = cls
        msg.x = pos[0]
        msg.y = pos[1]
        publisher.publish(msg)
        # print("x = " + str(pos[0]) + "\t y = " + str(pos[1]))
        # print("judge_pub success")
        
        
def pub_process(tracer):
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        for k in range(6):
            tracer[k].pub()
        # print("="*20)
        # print(onTracing, end="\n")
        # 打印摸个目标是否丢失的信息
        # print("目标是否丢失：")
        # print(bLeft)
        # 打印追踪这的状态机
        # print("状态机：")
        # print(tracer[0].override, tracer[1].override,tracer[2].override,tracer[3].override,tracer[4].override,tracer[5].override)
        # 打印追踪的目标点数据
        # print("追踪的目标点：")
        # print(tracer[0].trace_loc, tracer[1].trace_loc,tracer[2].trace_loc,tracer[3].trace_loc,tracer[4].trace_loc,tracer[5].trace_loc)
        rate.sleep()
            

if __name__ == "__main__":
    tracer = []
    # 参数 anonymous 的作用是节点不需要一个唯一的名字，ROS master 会自动为它生成一个名字
    rospy.init_node("Target_Position_Calculator", anonymous=True)

    # 订阅了一个丢失的角色话题
    rospy.Subscriber("/left_actors",String, UpdateCapture)

    # 初始化typhoon, 开始追踪
    for k in range(6):
        tracer.append(Tracer(k))
    # listener = Listener(0)
    # init_pos = np.array([0,-3,0])
    
    rospy.sleep(1.5)
    threads = []
    # 把无人机接受数据的话题加入线程中
    for k in range(6):
        t = threading.Thread(target=tracer[k].sync_process)
        threads.append(t)
        t.start()
    t = threading.Thread(target=pub_process, kwargs={"tracer" : tracer})
    threads.append(t) 
    t.start()
    # 启动线程
    for t in threads:
        t.join()