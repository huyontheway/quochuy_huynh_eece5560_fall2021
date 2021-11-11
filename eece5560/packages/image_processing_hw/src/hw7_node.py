#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class HW7:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.callback)
        
        self.pub_crop = rospy.Publisher("image_cropped", Image, queue_size=10)
        self.pub_white = rospy.Publisher("image_white", Image, queue_size=10)
        self.pub_yellow = rospy.Publisher("image_yellow", Image, queue_size=10)
    
    def callback(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Get image dimensions
        self.height = cv_img.shape[0]
        self.width = cv_img.shape[1]
        
        # Crop image
        crop_img = cv_img[self.height//2:self.height, 0:self.width].copy()
        ros_crop = self.bridge.cv2_to_imgmsg(crop_img, "bgr8")
        self.pub_crop.publish(ros_crop)
        
        # Filter white image
        image_hsv = cv2.cvtColor(crop_img,cv2.COLOR_BGR2HSV)
        img_white = cv2.inRange(image_hsv,(0,0,20),(180,25,255))  
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_erode = cv2.erode(img_white, kernel)
        
        ros_white = self.bridge.cv2_to_imgmsg(image_erode, "mono8")
        self.pub_white.publish(ros_white)
        
        # Filter yellow image         
        img_yellow = cv2.inRange(image_hsv,(28,150,20),(32,255,255))        
        ros_yellow = self.bridge.cv2_to_imgmsg(img_yellow, "mono8")
        self.pub_yellow.publish(ros_yellow)
        

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("hw7_node", anonymous=True)
    HW7()
    rospy.spin()
