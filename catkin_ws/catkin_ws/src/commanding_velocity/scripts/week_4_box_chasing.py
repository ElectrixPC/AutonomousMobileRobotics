# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:08:04 2018

@author: user
"""

#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist



class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        cv2.namedWindow("mask", 1)

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = numpy.array([ 50, 100,  50])
        upper_green = numpy.array([ 180, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        h, w, d = image.shape
    
        # calculate the area of the values that are within the mask
        M = cv2.moments(mask)
        # if area over zero for the mask
        if M['m00'] > 0:
          # find the centre of the area
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          # make a circle of the centre and add it to the image
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          # calculate the error by subtracting the location of centre of red dot on x axis
          # by the width of the image/2 (to get the left and right values)
          err = cx - w/2
          self.twist.linear.x = 0.2 # set speed 
          self.twist.angular.z = -float(err) / 100 # set turn based on the amount of error from angle of current turtlebot
          self.cmd_vel_pub.publish(self.twist)
          # END CONTROL
        cv2.imshow("window", image)
        cv2.imshow("mask", mask)
        cv2.waitKey(1)

rospy.init_node('follower')
follower = Follower()
rospy.spin()