#!/usr/bin/env python3

import rospy
from mystery_package.msg import UnitsLabelled


class HW3Node:
    def __init__(self):
        rospy.Subscriber("/mystery/output2", UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher("conversion_result", UnitsLabelled, queue_size=10)
        self.total = 0
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"

    def callback(self, msg):
        self.total = msg.value * 3.2808
        self.pub_msg.value = self.total
        self.pub_units.publish(self.pub_msg)


if __name__ == '__main__':
    rospy.init_node('hw3_node',anonymous=True)
    HW3Node()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

