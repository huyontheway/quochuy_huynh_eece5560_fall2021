#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from hw9_package.src import pid_class
from pid_class import PID_Control


class HW9Node:
    def __init__(self):
        rospy.Subscriber("error", Float32, self.callback)
        self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
        
        self.pid = PID_Control()
        self.pid.k_p = 0.01
        self.pid.k_i = 0
        self.pid.k_d = 0.31

    def callback(self, msg):
        self.pid.current_e = msg.data
        rospy.loginfo(self.pid.current_e)
        self.result = self.pid.calculate_PID()
        self.pub.publish(self.result)


if __name__ == '__main__':
    rospy.init_node('hw9_node',anonymous=True)
    HW9Node()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

