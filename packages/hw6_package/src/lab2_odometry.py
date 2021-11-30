#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import Pose2D

class Lab2_Odometry:
    def __init__(self):

        #Create subscribers
        self.sub_left = rospy.Subscriber("/huyduckiebot/left_wheel_encoder_node/tick",WheelEncoderStamped,self.left_wheel)

        self.sub_right = rospy.Subscriber("/huyduckiebot/right_wheel_encoder_node/tick",WheelEncoderStamped,self.right_wheel)

        #Create publisher
        self.pub = rospy.Publisher("current_pose",Pose2D, queue_size = 10)
        
        self.left_tick = 0
        self.right_tick = 0
        self.delta_left_tick = 0
        self.delta_right_tick = 0
        self.prev_left_tick = 0
        self.prev_right_tick = 0
        self.delta_s_l = 0
        self.delta_s_r = 0
        self.wheelRadius = 0.0318
        self.t_per_rev = 135
        self.baseline = 0.1 #between wheels
        self.delta_s = 0
        self.delta_theta = 0
        self.delta_x = 0
        self.delta_y = 0
        self.new_x = 0
        self.new_y = 0
        self.new_theta = 0
        self.prev_x = 0
        self.prev_y = 0
        self.prev_theta = 0
        self.pub_pose = Pose2D()
        self.pub_pose.x = 0
        self.pub_pose.y = 0
        self.pub_pose.theta = 0
        self.begin_tick_l = 0
        self.begin_tick_r = 0
        self.first_l = True
        self.first_r = True
        
        
        
    def left_wheel(self,msg):
        if self.first_l == True:
            self.begin_tick_l = msg.data
            self.left_tick = msg.data - self.begin_tick_l
            self.first_l = False
            #rospy.loginfo("Left wheel got %d", self.left_tick)
        
        else:
            #Grab how many left wheel ticks have occurred
            self.left_tick = msg.data - self.begin_tick_l
            #rospy.loginfo("Left wheel got %d", self.left_tick)


    def right_wheel(self,msg):
        if self.first_r == True:
            self.begin_tick_r = msg.data
            self.right_tick = msg.data - self.begin_tick_r
            self.first_r = False
            #rospy.loginfo("Right wheel got %d", self.right_tick)
        
        else:
            #Grab how many left wheel ticks have occurred
            self.right_tick = msg.data - self.begin_tick_r
            #rospy.loginfo("Right wheel got %d", self.right_tick)

        #rospy.loginfo("Right wheel got %d", msg.data)
        
    def calc(self):
        self.delta_left_tick = self.left_tick - self.prev_left_tick
        self.prev_left_tick = self.left_tick
        
        self.delta_right_tick = self.right_tick - self.prev_right_tick
        self.prev_right_tick = self.right_tick
        
        self.delta_s_l = (2 * math.pi * self.wheelRadius * self.delta_left_tick)/self.t_per_rev
        self.delta_s_r = (2 * math.pi * self.wheelRadius * self.delta_right_tick)/self.t_per_rev
        
        #Calculate new x, y and theta
        self.delta_s = (self.delta_s_l + self.delta_s_r)/2
        self.delta_theta = (self.delta_s_r - self.delta_s_l)/self.baseline
        
        self.delta_x = self.delta_s * math.cos(self.prev_theta + (self.delta_theta/2))
        self.delta_y = self.delta_s * math.sin(self.prev_theta + (self.delta_theta/2))
        
        self.new_x = self.prev_x + self.delta_x
        self.prev_x = self.new_x
        self.new_y = self.prev_y + self.delta_y
        self.prev_y = self.new_y
        self.new_theta = self.prev_theta + self.delta_theta
        self.prev_theta = self.new_theta
        
        self.pub_pose.x = self.new_x
        self.pub_pose.y = self.new_y
        self.pub_pose.theta = self.new_theta
        
        #rospy.loginfo("Radius %lf", self.wheelRadius)
        rospy.loginfo("Pose x is %lf", self.new_x)

        rospy.loginfo("Pose y is %lf", self.new_y)

        rospy.loginfo("Pose theta is %lf", self.new_theta * 180 / math.pi)
        
        self.pub.publish(self.pub_pose)
        
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('lab2_odometry', anonymous = True)
        odometry = Lab2_Odometry()
        rate=rospy.Rate(5) # 5hz
        
        while not rospy.is_shutdown():
            odometry.calc()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
