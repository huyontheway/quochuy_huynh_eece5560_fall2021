#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32


class HW4Node:
    def __init__(self):
        rospy.Subscriber("input", Float32, self.callback)
        self.pub = rospy.Publisher("/mystery/input", Float32, queue_size=10)
        self.value = 0
        self.convert = 0

    def callback(self, msg):
        if rospy.has_param("feet"):
            self.convert = rospy.get_param("feet")
            self.value = msg.data * self.convert
        elif rospy.has_param("smoots"):
            self.convert = rospy.get_param("smoots")
            self.value = msg.data * self.convert
        else:
            self.value = msg.data
        self.pub.publish(self.value)


if __name__ == '__main__':
    rospy.init_node('hw4_node',anonymous=True)
    HW4Node()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

