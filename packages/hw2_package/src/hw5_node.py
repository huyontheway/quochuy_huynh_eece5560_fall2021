#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Vector2D
import numpy as np

R_T_S = np.array([[-1,0,-2], [0,-1,0], [0,0,1]])

class HW5Node:

    def __init__(self):
        rospy.Subscriber("input_pos", Vector2D, self.callback)
        self.pub = rospy.Publisher("robot_coor_pos", Vector2D, queue_size=10)        
        self.pub_msg1 = Vector2D()
        

    def callback(self, msg):
        self.S_P_a = np.array([[msg.x],[msg.y],[1]])
        self.R_P_a = np.dot(R_T_S,self.S_P_a)
        self.pub_msg1.x = self.R_P_a[0][0]
        self.pub_msg1.y = self.R_P_a[1][0]
        self.pub.publish(self.pub_msg1)


if __name__ == '__main__':
    rospy.init_node('hw5_node',anonymous=True)
    HW5Node()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()    
        

