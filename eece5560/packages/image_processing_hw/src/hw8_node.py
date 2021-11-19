#!/usr/bin/env python3

import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HW8:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image_cropped", Image, self.get_crop)
        rospy.Subscriber("image_white", Image, self.get_white)
        rospy.Subscriber("image_yellow", Image, self.get_yellow)
        
        self.pub = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.pub_edge_crop = rospy.Publisher("image_edges", Image, queue_size=10)
        self.pub_white_edge = rospy.Publisher("white_edge", Image, queue_size=10)
        self.pub_yellow_edge = rospy.Publisher("yellow_edge", Image, queue_size=10)
        
        self.crop_img_cond = False
        self.white_img_cond = False
        self.yellow_img_cond = False
        
    def get_crop(self, msg):
        self.crop_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.crop_img_cond = True
        
    def get_white(self, msg):
        self.white_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.white_img_cond = True
        
    def get_yellow(self, msg):
        self.yellow_img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.yellow_img_cond = True
        
        if self.crop_img_cond == True and self.white_img_cond == True and self.yellow_img_cond == True:
            # Edge detection using Canny with the crop image
            self.gray_crop = cv2.cvtColor(self.crop_img,cv2.COLOR_BGR2GRAY)
            
            self.sigma = 0.3
            self.median = np.median(self.gray_crop)
            self.lower = int(max(0, (1.0 - self.sigma) * self.median))
            self.upper = int(min(255, (1.0 + self.sigma) * self.median))
            
            self.crop_canny = cv2.Canny(self.gray_crop,self.lower,self.upper)
            self.crop_img_edge = self.bridge.cv2_to_imgmsg(self.crop_canny, "mono8")
            self.pub_edge_crop.publish(self.crop_img_edge)
            
            # Bitwise between the crop image with edge detection and filtered image with white line
            self.output_image_white = cv2.bitwise_and(self.white_img, self.crop_canny)
            self.white_img = self.bridge.cv2_to_imgmsg(self.output_image_white, "mono8")
            self.pub_white_edge.publish(self.white_img)
            
            # Bitwise between the crop image with edge detection and filtered image with yellow line
            self.output_image_yellow = cv2.bitwise_and(self.yellow_img, self.crop_canny)
            self.yellow_img = self.bridge.cv2_to_imgmsg(self.output_image_yellow, "mono8")
            self.pub_yellow_edge.publish(self.yellow_img)
            
            # Hough transform
            self.hough_white = cv2.HoughLinesP(self.output_image_white,5,math.pi/180.0, 40, np.array([]),0,0)
            self.hough_yellow = cv2.HoughLinesP(self.output_image_yellow,5,math.pi/180.0,40,np.array([]),0,0)
            
            self.crop_copy = np.copy(self.crop_img)
            
            if self.hough_white is not None:
                for i in range(len(self.hough_white)):
                    self.line_white = self.hough_white[i][0]
                    cv2.line(self.crop_copy, (self.line_white[0],self.line_white[1]), (self.line_white[2],self.line_white[3]), (255,0,0),2,cv2.LINE_AA)
                    cv2.circle(self.crop_copy, (self.line_white[0],self.line_white[1]), 2, (0,255,0))
                    cv2.circle(self.crop_copy, (self.line_white[2],self.line_white[3]), 2, (0,255,0))
            
            if self.hough_yellow is not None:   
                for i in range(len(self.hough_yellow)):
                    self.line_yellow = self.hough_yellow[i][0]
                    cv2.line(self.crop_copy, (self.line_yellow[0],self.line_yellow[1]), (self.line_yellow[2],self.line_yellow[3]), (255,0,0),2,cv2.LINE_AA)
                    cv2.circle(self.crop_copy, (self.line_yellow[0],self.line_yellow[1]), 2, (0,255,0))
                    cv2.circle(self.crop_copy, (self.line_yellow[2],self.line_yellow[3]), 2, (0,255,0))
                    
                self.output = self.bridge.cv2_to_imgmsg(self.crop_copy, "bgr8")
                self.pub.publish(self.output)
        
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("hw8_node", anonymous=True)
    HW8()
    rospy.spin()
