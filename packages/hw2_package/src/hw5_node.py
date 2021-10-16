#!/usr/bin/env python3

import rospy
import numpy as np
import math
from duckietown_msgs.msg import Vector2D

R_T_S = np.array([[-1,0,-2], [0,-1,0], [0,0,1]])   # define the transformation matrix between robot and sensor coordinates
W_T_R = np.array([[-1/math.sqrt(2),-1/math.sqrt(2),2], [1/math.sqrt(2),-1/math.sqrt(2),7], [0,0,1]]) # define the transformation matrix between robot and world coordinates

class HW5Node:
    def __init__(self):
        rospy.Subscriber("input_pos", Vector2D, self.callback)
        self.pub1 = rospy.Publisher("robot_coor_pos", Vector2D, queue_size=10)       
        self.pub2 = rospy.Publisher("world_coor_pos", Vector2D, queue_size=10) 
         
        self.pub_msg_robot = Vector2D()
        self.pub_msg_world = Vector2D()
        self.S_P_a = np.array([[0], [0], [0]])
        self.R_P_a = np.array([[0], [0], [0]])
        self.W_P_a = np.array([[0], [0], [0]])
        
        self.pub_msg_robot.x = 0
        self.pub_msg_robot.y = 0
        self.pub_msg_world.x = 0
        self.pub_msg_world.y = 0
        

    def callback(self, msg):
        self.S_P_a = np.array([[msg.x], [msg.y], [1]])
        self.R_P_a = np.dot(R_T_S,self.S_P_a)
        self.W_P_a = np.dot(W_T_R,self.R_P_a)
        
        self.pub_msg_robot.x = self.R_P_a[0][0]
        self.pub_msg_robot.y = self.R_P_a[1][0]
        
        self.pub_msg_world.x = self.W_P_a[0][0]
        self.pub_msg_world.y = self.W_P_a[1][0]
        
        self.pub1.publish(self.pub_msg_robot)
        self.pub2.publish(self.pub_msg_world)


if __name__ == '__main__':
    rospy.init_node('hw5_node',anonymous=True)
    HW5Node()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()    
        

