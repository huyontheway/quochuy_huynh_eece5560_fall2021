#!/usr/bin/env python3

import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from duckietown_msgs.msg import Segment, SegmentList

class Lab3:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("/huyduckiebot/camera_node/image/compressed", CompressedImage, self.lanefilter, queue_size=1, buff_size=2**24)
        self.pub_hough = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.pub_ground = rospy.Publisher("/huyduckiebot/line_detector_node/segment_list", SegmentList, queue_size=10)
        
        
    def lanefilter(self, msg):    
        # convert to a ROS image using the bridge
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        image_size = (160, 120)
        offset = 40
        new_image = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        cropped_image = new_image[offset:, :]
        
        # Filter white image
        image_hsv = cv2.cvtColor(cropped_image,cv2.COLOR_BGR2HSV)
        img_white = cv2.inRange(image_hsv,(0,0,20),(180,35,255))  
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_erode_white = cv2.erode(img_white, kernel)
        image_dilate_white = cv2.dilate(image_erode_white, kernel)
        
        # Filter yellow image         
        img_yellow = cv2.inRange(image_hsv,(24,80,20),(35,255,255))  
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_erode_yellow = cv2.erode(img_yellow, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        image_dilate_yellow = cv2.dilate(image_erode_yellow, kernel) 
        
        # Edge detection using Canny with the crop image
        self.gray_crop = cv2.cvtColor(cropped_image,cv2.COLOR_BGR2GRAY)
        
        #self.sigma = 0.3
        #self.median = np.median(self.gray_crop)
        #self.lower = int(max(0, (1.0 - self.sigma) * self.median))
        #self.upper = int(min(255, (1.0 + self.sigma) * self.median))
        
        self.crop_canny = cv2.Canny(self.gray_crop,80,210)
        
        # Bitwise between the crop image with edge detection and filtered image with white line
        self.output_image_white = cv2.bitwise_and(image_dilate_white, self.crop_canny)
        
        # Bitwise between the crop image with edge detection and filtered image with yellow line
        self.output_image_yellow = cv2.bitwise_and(image_dilate_yellow, self.crop_canny)
        
        # Hough transform
        self.hough_white = cv2.HoughLinesP(self.output_image_white,7,math.pi/180.0, 40,0.02,1)
        self.hough_yellow = cv2.HoughLinesP(self.output_image_yellow,5,math.pi/180.0,40,0.02,1)
        
        self.crop_copy = np.copy(cropped_image)
        
        if self.hough_white is not None:
            for i in range(len(self.hough_white)):
                rospy.loginfo("Drawing white lines")
                self.line_white = self.hough_white[i][0]
                cv2.line(self.crop_copy, (self.line_white[0],self.line_white[1]), (self.line_white[2],self.line_white[3]), (255,255,150),2,cv2.LINE_AA)
                cv2.circle(self.crop_copy, (self.line_white[0],self.line_white[1]), 2, (0,0,0))
                cv2.circle(self.crop_copy, (self.line_white[2],self.line_white[3]), 2, (0,0,0))
        
        if self.hough_yellow is not None:   
            for i in range(len(self.hough_yellow)):
                rospy.loginfo("Drawing yellow lines")
                self.line_yellow = self.hough_yellow[i][0]
                cv2.line(self.crop_copy, (self.line_yellow[0],self.line_yellow[1]), (self.line_yellow[2],self.line_yellow[3]), (0,255,204),2,cv2.LINE_AA)
                cv2.circle(self.crop_copy, (self.line_yellow[0],self.line_yellow[1]), 2, (0,0,0))
                cv2.circle(self.crop_copy, (self.line_yellow[2],self.line_yellow[3]), 2, (0,0,0))
                
            self.output = self.bridge.cv2_to_imgmsg(self.crop_copy, "bgr8")
            self.pub_hough.publish(self.output)
            
                
        #Normalize image
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[0], 1. / image_size[1], 1. / image_size[0], 1. / image_size[1]])
        
        self.pub_segmentlist = SegmentList()
        
        #Normalize lines
        if self.hough_white is not None:   
            self.hough_white = (self.hough_white + arr_cutoff) * arr_ratio
            
        if self.hough_yellow is not None:   
            self.hough_yellow = (self.hough_yellow + arr_cutoff) * arr_ratio
        
        if self.hough_white is not None:
            for line1 in self.hough_white:
                self.segment = Segment()
                
                #self.array_sum = np.sum(line1[0])
                #self.array_has_nan = np.isnan(self.array_sum)
                #if self.array_has_nan == True:
                #    rospy.loginfo("Found NaN")
                #    continue
                
                x1,y1,x2,y2 = line1[0]
                
                #x1 = np.nan_to_num(x1)

                #y1 = np.nan_to_num(y1)

                #x2 = np.nan_to_num(x2)

                #y2 = np.nan_to_num(y2) 
                self.segment.pixels_normalized[0].x = x1
                self.segment.pixels_normalized[0].y = y1
                self.segment.pixels_normalized[1].x = x2
                self.segment.pixels_normalized[1].y = y2
                    
                self.segment.color = 0
                    
                self.pub_segmentlist.segments.append(self.segment)
                
        if self.hough_yellow is not None:
            for line2 in self.hough_yellow:
                self.segment = Segment()
                
                #self.array_sum = np.sum(line2[0])
                #self.array_has_nan = np.isnan(self.array_sum)
                #if self.array_has_nan == True:
                #    rospy.loginfo("Found NaN")
                #    continue
                
                x1,y1,x2,y2 = line2[0]
                
                #x1 = np.nan_to_num(x1)

                #y1 = np.nan_to_num(y1)

                #x2 = np.nan_to_num(x2)

                #y2 = np.nan_to_num(y2) 
                self.segment.pixels_normalized[0].x = x1
                self.segment.pixels_normalized[0].y = y1
                self.segment.pixels_normalized[1].x = x2
                self.segment.pixels_normalized[1].y = y2
                self.segment.color = 1
                    
                self.pub_segmentlist.segments.append(self.segment)
                rospy.loginfo("Publishing segments to ground projection.")
                self.pub_ground.publish(self.pub_segmentlist) 
                
        
        #if self.hough_yellow is not None or self.hough_white is not None:
        #    rospy.loginfo("Publishing segments to ground projection.")
        #    self.pub_ground.publish(self.pub_segmentlist)  
            
                  
if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("lab3_node", anonymous=True)
    Lab3()
    rospy.spin()
