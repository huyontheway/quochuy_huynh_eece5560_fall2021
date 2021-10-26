#!/usr/bin/env python3

import rospy
import math
from odometry_hw.msg import DistWheel
from odometry_hw.msg import Pose2D


class HW6Node:
    def __init__(self):
        rospy.Subscriber("/dist_wheel", DistWheel, self.callback)
        self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)
        
        self.pub_pose = Pose2D()
        self.pub_pose.x = 0
        self.pub_pose.y = 0
        self.pub_pose.theta = 0

    def callback(self, msg):
        self.delta_s = (msg.dist_wheel_left + msg.dist_wheel_right) / 2
        self.delta_theta = (msg.dist_wheel_right - msg.dist_wheel_left) / 0.1
        
        self.delta_x = self.delta_s * math.cos(self.pub_pose.theta + (self.delta_theta / 2))
        self.delta_y = self.delta_s * math.sin(self.pub_pose.theta + (self.delta_theta / 2))
        
        self.pub_pose.x = self.pub_pose.x + self.delta_x
        self.pub_pose.y = self.pub_pose.y + self.delta_y
        self.pub_pose.theta = self.pub_pose.theta + self.delta_theta
        
        self.pub.publish(self.pub_pose)


if __name__ == '__main__':
    rospy.init_node('hw6_node',anonymous=True)
    HW6Node()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

