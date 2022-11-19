#!/usr/bin/env python2

R2D = 180/3.141592
import sensor_msgs.msg
from sensor_msgs.msg import JointState

import rospy
from std_msgs.msg import Time
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
import motor
import kinematics
from nav_msgs.msg import Odometry


motor = motor.Motor()

class Control():
    def __init__(self):
        print("Control_Start!!!")
        self.mode = "Go_home"
        rospy.init_node('Control_node', anonymous=True)
        rospy.Subscriber("/joint_states",  JointState, self.Trajectory_callback)
        rospy.Subscriber('/timer', Time, self.Time_call)
        rospy.Subscriber('/Decision_mode',String, self.Decision_mode_call)
        self.pub_xyz_hardware = rospy.Publisher('/xyz_hardware', Odometry, queue_size = 1)
        self.pub_is_it_moving = rospy.Publisher('/is_it_tracking', Bool, queue_size = 1)
            
    
    def Trajectory_callback(self, data):
        motor.degree(data.position[0]*R2D, data.position[1]*R2D, data.position[2]*R2D, data.position[3]*R2D, data.position[4]*R2D)

    def Time_call(self,time):
        self.pub_is_it_moving.publish(motor.Is_it_tracking())

        if self.mode == "Go_home" :
            print("mode : Go_home")

        elif self.mode == "Searching":
            print("mode : Searching")

        elif self.mode == "Start_tracking"

        elif self.mode == "Tracking":
            print("mode : Tracking")

        elif self.mode == "Grip":
            print("mode : Grip")

        elif self.mode == "Go_home_grip":
            print("mode : Go_homoe_grip")

        elif self.mode == "Let_it_go":
            print("mode : Let_it_go")
        

    def Decision_mode_call(self,data):
        self.mode = data.data

if __name__ == '__main__':
    control = Control()
    rospy.spin()

