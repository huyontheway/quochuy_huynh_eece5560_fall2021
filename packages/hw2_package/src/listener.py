#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Listener:
    def __init__(self):
        rospy.Subscriber("/mystery/output1", Float32, self.callback)
        

    def callback(self,msg):
        
        if rospy.has_param("/homework4/feet"):
            rospy.loginfo("/mystery/output1 published %f feet", msg.data)

        elif rospy.has_param("/homework4/smoots"):
            rospy.loginfo("/mystery/output1 published %f smoots", msg.data)

        else:
            rospy.loginfo("/mystery/output1 published %f meters", msg.data)


if __name__ == '__main__':
    rospy.init_node('listener',anonymous=True)
    Listener()

rospy.spin()
