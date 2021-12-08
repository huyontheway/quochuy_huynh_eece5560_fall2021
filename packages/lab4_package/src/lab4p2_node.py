#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped,LanePose
from lab4_package.src import pid_class
from pid_class import PID_Control


class Lab4_Part2:
    def __init__(self):
        rospy.Subscriber("/huyduckiebot/lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("/huyduckiebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        self.pub_cmd = Twist2DStamped()
        self.pub_cmd.v = 0.25
        self.pub_cmd.omega = 0
        
        # Initialize PID control for d
        self.pid_d = PID_Control()
        self.pid_d.k_p = 0
        self.pid_d.k_i = 0
        self.pid_d.k_d = 0
        
        # Initialize PID control for phi
        self.pid_phi = PID_Control()
        self.pid_phi.k_p = 0
        self.pid_phi.k_i = 0
        self.pid_phi.k_d = 0
        
    def callback(self, msg):
        # Set gains for each PID controller from param
        if rospy.has_param("k_p_d"):
            self.value = rospy.get_param("k_p_d")
            self.pid_d.k_p = self.value
            
        if rospy.has_param("k_i_d"):
            self.value = rospy.get_param("k_i_d")
            self.pid_d.k_i = self.value
            
        if rospy.has_param("k_p_phi"):
            self.value = rospy.get_param("k_p_phi")
            self.pid_phi.k_p = self.value
        
        if rospy.has_param("k_i_phi"):
            self.value = rospy.get_param("k_i_phi")
            self.pid_phi.k_i = self.value
        
        
        # Get the current error for d = d_ref - d
        self.pid_d.current_e = msg.d_ref - msg.d
        
        # Get the current error for phi = phi_ref - phi
        self.pid_phi.current_e = msg.phi_ref - msg.phi
        
        rospy.loginfo("Error in d")
        rospy.loginfo(self.pid_d.current_e)
        rospy.loginfo("Error in phi")
        rospy.loginfo(self.pid_phi.current_e)
        
        # Calculate result from PID
        self.pub_cmd.omega = self.pid_d.calculate_PID() + self.pid_phi.calculate_PID()
        
        rospy.loginfo("Omega is:")
        rospy.loginfo(self.pub_cmd.omega)
       
        # Publish new velocity and andular velocity 
        self.pub.publish(self.pub_cmd)

if __name__ == '__main__':
    rospy.init_node('lab4p2_node',anonymous=True)
    Lab4_Part2()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

