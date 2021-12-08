#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class PID_Control():
    def __init__(self):
        self.k_p = 0
        self.k_i = 0
        self.k_d = 0
        self.p = 0
        self.i = 0
        self.d = 0
        self.result = 0
        self.current_e = 0
        self.prev_e = 0
        self.current_time = 0
        self.prev_time = 0
        self.error_sum = 0
    
    def calculate_PID(self):
        self.current_time = rospy.get_time()
        
        # Calculate pproportinoal
        self.p = self.k_p * self.current_e
        
        #Calculate integral
        #self.error_sum += self.current_e * (self.current_time - self.prev_time)
        self.i = self.i + self.current_e * (self.current_time - self.prev_time)
        #self.i = self.i + (self.k_i * self.p)
        
        #Calculate derivative
        self.d = self.k_d * (self.current_e - self.prev_e)/(self.current_time - self.prev_time )
        
        #Result
        self.result = self.p + self.k_i * self.i + self.d
        
        self.prev_e = self.current_e
        self.prev_time = self.current_time
        
        return self.result
