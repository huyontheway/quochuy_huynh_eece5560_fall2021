#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

x = 0 # Define a global variable named "x" and initialize with a value of 0
y = 1 # Define a global variable named "y" and initialize with a value of 1
z = 0 # Define a global variable named "z" and initialize with a value of 0

class Talker:

    def __init__(self):
       self.pub = rospy.Publisher('/mystery/input', Float32, queue_size=10)

    def talk(self, x, y):
        z = x + y
#        rospy.loginfo(z)
        self.pub.publish(z)
        return z


if __name__ == '__main__':
    try:
        rospy.init_node('talker',anonymous=True)
        t=Talker()
        rate = rospy.Rate(1) #1HZ
        while not rospy.is_shutdown():
            z = t.talk(x, y)  # calculate z from function talk
            x = y  # set x equal to y
            y = z  # set y equal to z
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

