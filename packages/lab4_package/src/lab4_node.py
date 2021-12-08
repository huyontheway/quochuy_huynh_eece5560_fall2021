#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped,AprilTagDetectionArray,FSMState
from lab4_package.src import pid_class
from pid_class import PID_Control


class Lab4:
    def __init__(self):
        rospy.Subscriber("/huyduckiebot/apriltag_detector_node/detections", AprilTagDetectionArray, self.callback)
        rospy.Subscriber("/huyduckiebot/fsm_node/mode", FSMState, self.callback_fsm)
        self.pub = rospy.Publisher("/huyduckiebot/lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        
        self.pub_cmd = Twist2DStamped()
        self.pub_cmd.v = 0
        self.pub_cmd.omega = 0
        
        # Initialize PID control for z direction
        self.pid_z = PID_Control()
        self.pid_z.k_p = 0.23
        self.pid_z.k_i = 0.0
        self.pid_z.k_d = 0.15
        
        # Initialize PID control for x direction
        self.pid_x = PID_Control()
        self.pid_x.k_p = 2.2
        self.pid_x.k_i = 0.0
        self.pid_x.k_d = 0.4
        
        self.z_desire = 0.1
        self.x_desire = 0.0
        
        self.lane_mode = False # Condition of the lane following mode to start/stop the robot
        self.tag_mode = False # Condition of the apriltag mode
    
    def callback_fsm(self, msg):
        if msg.state == "LANE_FOLLOWING" and self.lane_mode == False:
            self.tag_mode = True
            self.lane_mode = True
        elif msg.state == "NORMAL_JOYSTICK_CONTROL" and self.lane_mode == True:
            self.lane_mode = False   
            self.tag_mode = False
            
        rospy.loginfo("Tag mode is")
        rospy.loginfo(self.tag_mode)
        
    def callback(self, msg):
        if self.tag_mode == True:
            if len(msg.detections) != 0:
                rospy.loginfo("There's tag detected!")
                
                # PID for velocity in z direction
                # Get the current error = desired z location - current location of tag in z dir
                self.pid_z.current_e = self.z_desire - msg.detections[0].transform.translation.z
                
                rospy.loginfo("Error in z")
                rospy.loginfo(-self.pid_z.current_e)
                
                # Calculate result from PID
                self.pub_cmd.v = - self.pid_z.calculate_PID()
                
                rospy.loginfo("Speed")
                rospy.loginfo(self.pub_cmd.v)
                
                # PID for velocity in x direction
                # Get the current error = desired x location - current location of tag in x dir
                self.pid_x.current_e = self.x_desire - msg.detections[0].transform.translation.x
                
                rospy.loginfo("Error in x")
                rospy.loginfo(self.pid_x.current_e)
                
                # Calculate result from PID
                self.pub_cmd.omega = self.pid_x.calculate_PID()
                
                rospy.loginfo("Angular speed")
                rospy.loginfo(self.pub_cmd.omega)
               
                # Publish new velocity and andular velocity 
                self.pub.publish(self.pub_cmd)
                
            else: # If no tag is detected
                self.pub_cmd.v = 0 # Set velocity to be 0
                self.pub_cmd.omega = 0 # Set angular velocity to 0
                
                rospy.loginfo("No tag is detected")
                rospy.loginfo("Speed")
                rospy.loginfo(self.pub_cmd.v)
                rospy.loginfo("Angular speed")
                rospy.loginfo(self.pub_cmd.omega)

                # Publish new velocity and angular velocity
                self.pub.publish(self.pub_cmd)

if __name__ == '__main__':
    rospy.init_node('lab4_node',anonymous=True)
    Lab4()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

