# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:08:04 2018

@author: user
"""

#!/usr/bin/env python

import rospy
import cv2
import numpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        #self.image_sub = rospy.Subscriber("/turtlebot_1/usb_cam/image_raw",
        #                                  Image, self.callback)
        self.wheel_sub = rospy.Subscriber("/wheel_vel_left",
                                          Float32, self.callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.wheel_radius = 1
        self.robot_radius = 1

    # computing the forward kinematics for a differential drive
    def forward_kinematics(self, w_l, w_r):
        c_l = self.wheel_radius * w_l
        c_r = self.wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_r - c_l) / (2 * self.robot_radius)
        return (v, a)    
    
    def callback(self, data):
        print data
        
        v, a = self.forward_kinematics(data.data, 0.0)
        self.twist.linear.x = v
        self.twist.angular.z = a
        self.cmd_vel_pub.publish(self.twist)
        
        #try:
        #    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #except CvBridgeError, e:
        #    print e
        
        #bgr_thresh = cv2.inRange(cv_image,
        #                         numpy.array((2, 10, 2)),
        #                         numpy.array((2, 255, 2)))

        #hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #hsv_thresh = cv2.inRange(hsv_img,
        #                         numpy.array((50, 100, 50)),
        #                         numpy.array((180, 255, 255)))

        #print numpy.mean(hsv_img[:, :, 0])
        #print numpy.mean(hsv_img[:, :, 1])
        #print numpy.mean(hsv_img[:, :, 2])

        #bgr_contours, hierachy = cv2.findContours(bgr_thresh.copy(),
        #                                          cv2.RETR_TREE,
        #                                          cv2.CHAIN_APPROX_SIMPLE)

        #hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
        #                                          cv2.RETR_TREE,
        #                                          cv2.CHAIN_APPROX_SIMPLE)
        #for c in hsv_contours:
        #    a = cv2.contourArea(c)
        #    if a > 100.0:
        #        cv2.drawContours(cv_image, c, -1, (255, 0, 0))
        #print '===='
        
        #means_string = "Means: \\n%s \ \ n%s \n%s" %(numpy.mean(hsv_img[:, :, 0]), numpy.mean(hsv_img[:, :, 1]), numpy.mean(hsv_img[:, :, 2]))

        #self.publisher(means_string)       
        
        #cv2.imshow("Image window", hsv_thresh)
        
    def publisher(self, a):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.loginfo(a)
        pub.publish(a)
        print 'Published >:D'

image_converter()
rospy.init_node('image_converter', anonymous=True)
rospy.spin()
cv2.destroyAllWindows()
