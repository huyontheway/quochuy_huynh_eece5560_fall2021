#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState


class Lab2Node:
    def __init__(self):
        rospy.Subscriber("/huyduckiebot/fsm_node/mode", FSMState, self.callback)
        self.pub = rospy.Publisher("/huyduckiebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        self.pub_cmd = Twist2DStamped()
        self.pub_cmd.v = 0
        self.pub_cmd.omega = 0
        self.mode = False
        
    def callback(self, msg):
        if msg.state == "LANE_FOLLOWING" and self.mode == False:
            if rospy.get_param("route") == "line":
                self.mode = True
                self.pub_cmd.v = 0.35
                self.pub_cmd.omega = 0
                self.pub.publish(self.pub_cmd)
                
                rospy.sleep(5.8)
                
                self.pub_cmd.v = 0
                self.pub.publish(self.pub_cmd)
                
            elif rospy.get_param("route") == "square":
                for x in range(4):
                    self.mode = True
                    self.pub_cmd.v = 0.35
                    self.pub_cmd.omega = 0
                    self.pub.publish(self.pub_cmd)
                    
                    rospy.sleep(5.6)
                    
                    self.pub_cmd.v = 0
                    self.pub.publish(self.pub_cmd)    
                    
                    rospy.sleep(1)
                    
                    self.pub_cmd.omega = 6.0
                    self.pub.publish(self.pub_cmd)
                    
                    rospy.sleep(0.6)
                    
                    self.pub_cmd.v = 0
                    self.pub_cmd.omega = 0
                    self.pub.publish(self.pub_cmd)    
                    
                    rospy.sleep(1)
                    
            elif rospy.get_param("route") == "circle":
                self.mode = True
                self.pub_cmd.v = 0.35 #Set v = 0.6 m/s
                self.pub_cmd.omega = 2.8 #omega = 6 rad/s
                self.pub.publish(self.pub_cmd)
                rospy.sleep(14.5)
                
                self.pub_cmd.v = 0
                self.pub_cmd.omega = 0
                self.pub.publish(self.pub_cmd) 
        
        elif msg.state == "NORMAL_JOYSTICK_CONTROL" and self.mode == True:
            self.mode = False
            
if __name__ == '__main__':
    rospy.init_node('lab2_straight',anonymous=True)
    Lab2Node()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
