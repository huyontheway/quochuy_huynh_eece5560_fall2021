#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Listener:
    def __init__(self):
        rospy.Subscriber("/mystery/output1", Float32, self.callback)

    def callback(self,msg):
        rospy.loginfo("/mystery/output1 published %f", msg.data)

if __name__ == '__main__':
    rospy.init_node('listener',anonymous=True)
    Listener()

rospy.spin()
