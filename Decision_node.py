#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from yolo_msg_pkg.msg import detection

# from custom_pkg.msg import D2C

class Decision():
    def __init__(self):
        rospy.init_node('Decision', anonymous=True)
        rospy.Subscriber('/timer', Time, self.Time_call)
        rospy.Subscriber('/is_it_tracking', Bool, self.Is_it_tracking)
        rospy.Subscriber('/xyz_sensor', detection, self.Odometry_sensing)
        rospy.Subscriber('/xyz_hardware', Odometry, self.Odometry_control)
        
        self.pub_mode = rospy.Publisher('/Decision_mode', String, queue_size=1)
        self.pub_xyz_target = rospy.Publisher('/xyz_target', Odometry, queue_size=1)
        self.x_loc = 0
        self.y_loc = 0
        self.z_loc = 0

        self.point_list = []

        
        self.x_global = -0.0764 #home 일 때 x_global 넣으면 될 듯?
        self.y_global = 0.02326 #home 일 때 y_global 넣으면 될 듯?
        self.z_glabal = 0.57 #home 일 때 z_global 넣으면 될 듯?
        self.mode = "Go_home"
        self.is_it_tracking = True # Tracking 중이야?

    def Time_call(self, time): # mode를 각각의 노드에 보내줘야함
        
        if self.mode == "Go_home" and self.is_it_tracking == True:
            self.mode = "Go_home"

        elif self.mode == "Go_home" and self.is_it_tracking == False:
            self.mode = "Searching"

        elif self.mode == "Searching":
            if self.Select_target() == True:
                self.mode = "Start_tracking"
            else:
                self.mode = "Searching"
        #how about adding searching_target?
        elif self.mode == "Start_tracking":
            if self.is_it_tracking == True:
                self.mode = "Tracking"

        elif self.mode == "Tracking" and self.is_it_tracking == False:
            self.mode = "Grip"

        elif self.mode == "Grip":
            if self.is_it_gripped == True:
                self.mode = "Go_home_grip"

        elif self.mode == "Go_home_grip":
            if self.is_it_tracking == False:
                self.mode = "Let_it_go"

        elif self.mode == "Let_it_go":
            if self.is_it_tracking == False and self.is_it_gripped == False:
                self.mode = "First_senario_end"

        self.pub_mode.publish(self.mode)
        print('mode : ' + self.mode)



    def Odometry_sensing(self, data): # sensing 이랑 연동해서 사용할 부분 많은 데이터 받도록 수정
        #sensing팀이 고치기. 리스트로 받아오기
        #self.point_list = []
        print("number of targets : ",data.len)
        if(data.len > 0):
            self.point_list = list(zip(data.X,data.Y,data.Z))
            self.point_list = self.point_list[:data.len]
        
        
        print(self.point_list)
    
    def Odometry_control(self, data):
        self.x_current = data.pose.pose.position.x
        self.y_current = data.pose.pose.position.y
        self.z_current = data.pose.pose.position.z

    def Select_target(self): ########################################################
        # you should deep copy self.point_list
        now_target_list = self.point_list[:] # (x,y,distance) 
        return 1
        ### 이 부분을 Odometry_sensing에서 받은 리스트들을 탐색하는 코드 작성해야한다.
        #sensing팀이 고치기
        # if ok: # workspace 안에 있는 target을 인식할 때
        #     return True 
        # else: # workspace 밖에 있는 target을 인식할 때
        #     return False


    def Calculation_XYZ(self):
        #김보겸이 작성 필요
        #x_current, y_current, z_current / x_loc, y_loc, z_loc / 필요하면 degree받아다가 계산해야함
        
        # pose = 
        self.x_global = 0.2
        self.y_global = 0.2
        self.z_global = 0.2
        
        # Odometry msg 형태로 담아서 publish 
        # 도착지점 : Trajectory_planning_node.py/XYZ_target_call()
        self.pub_xyz_target.publish()
    
    def Is_it_tracking(self, data):
        self.is_it_tracking = data




Dec = Decision()
rospy.spin()