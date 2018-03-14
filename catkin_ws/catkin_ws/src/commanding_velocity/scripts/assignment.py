# -*- coding: utf-8 -*-
"""
Created on Mon Feb 12 15:08:04 2018

@author: user
"""

#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from pprint import pformat
from tf_conversions import transformations
from math import pi
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        #cv2.namedWindow("depth", 1)
        cv2.namedWindow("mask", 1)        
        self.image_sub = rospy.Subscriber("turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.image_depth = rospy.Subscriber("turtlebot/scan", LaserScan, self.depthcallback)
        self.image_odom = rospy.Subscriber("/turtlebot/odom", Odometry, self.explore)         
        self.cmd_vel_pub = rospy.Publisher("turtlebot/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.centrePointX = 0
        self.centrePointY = 0
        self.depth = 100000000
        self.dataranges = 0
        self.firstTime = True
        self.explore = np.zeros((10000, 10000))
        self.found = False
        self.commandChanges = 0
        self.search = False
        self.command = ""
        self.seek = False
        self.founditer = 0
        self.searchtimes = 0 
        self.maskGreen = 0
        self.maskRed = 0
        self.maskYellow = 0
        self.maskBlue = 0
        self.spinTimes = 0
        self.mask = self.maskGreen + self.maskRed + self.maskYellow + self.maskBlue
        self.colors = ["red", "green", "yellow", "blue"] 
        
    
        
    def depthcallback(self, data):
        # Return is a raw value of closeness to bot - 0 being far away, 255 being super close
        
        #NewImg = self.bridge.imgmsg_to_cv2(data.ranges,"passthrough")
        #depth_array = np.array(NewImg, dtype=np.float32)
        #cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        self.dataranges = data
        if not self.centrePointX == 0:
            self.depth = data.ranges[self.centrePointX]
            if str(self.depth) == "nan":
                self.depth = 100000000
                
        self.right  = min(self.dataranges.ranges[:320]) + np.nanmean(self.dataranges.ranges[260:380])
        self.left   = min(self.dataranges.ranges[320:]) + np.nanmean(self.dataranges.ranges[320:380])
        #cv2.imshow("depth", depth_array)
        cv2.waitKey(1)
        #cv2.imwrite("depth.png", depth_array*255)

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([ 45, 100,  50])
        upper_green = np.array([ 75, 255, 255])
        
        lower_red = np.array([ 0, 200,  100])
        upper_red = np.array([ 5, 255, 255])
        
        lower_yellow = np.array([ 20, 200,  100])
        upper_yellow = np.array([ 50, 255, 195])
        
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([150, 255, 255])
        
        self.maskGreen  = cv2.inRange(hsv, lower_green,  upper_green)
        self.maskRed    = cv2.inRange(hsv, lower_red,    upper_red)
        self.maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        self.maskBlue   = cv2.inRange(hsv, lower_blue,   upper_blue)
        
        self.MG = cv2.moments(self.maskGreen)['m00']
        self.MR = cv2.moments(self.maskRed)['m00']
        self.MY = cv2.moments(self.maskYellow)['m00']
        self.MB = cv2.moments(self.maskBlue)['m00']        
        
        self.mask = None
        if "red" in self.colors:
            if self.mask == None:
                self.mask = self.maskRed
            self.mask += self.maskRed
        if "yellow" in self.colors:
            if self.mask == None:
                self.mask = self.maskYellow
            self.mask += self.maskYellow
        if "green" in self.colors:
            if self.mask == None:
                self.mask = self.maskGreen
            self.mask += self.maskGreen
        if "blue" in self.colors:
            if self.mask == None:
                self.mask = self.maskBlue
            self.mask += self.maskBlue
            
        h, w, d = image.shape
    
        # calculate the area of the values that are within the mask
        M = cv2.moments(self.mask)
        
        cv2.imshow("window", image)
        cv2.imshow("mask", self.mask)
        
        # if M['m00'] > 100000:
        #     if self.depth > 0.6:
        #         self.seekmode(image, M)
        #     else:
        #         self.foundmode(image, M)
        # else:
        #     self.searchmode(image, self.mask)
        

    def odom_orientation(self, q):
        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        return y * 180 / pi
        
    def explore(self, data):
        # NORMALISE TO ZERO AT START FOR DATA EASE
        if self.firstTime:
            self.diffToStart = [int(data.pose.pose.position.x * 1000), int(data.pose.pose.position.x * 1000)]
            self.firstTime = False
        currentPos = [x, y] - self.diffToStart

        depth = self.depth
        angle = self.odom_orientation(data.pose.pose.orientation)
        #UP
        if angle > 300 and angle < 60: 
            endPos = [x, y + depth]
        #RIGHT
        if angle > 50 and angle < 130: 
            endPos = [x + depth, y] 
        #DOWN
        if angle > 130 and angle < 220: 
            endPos = [x, y - depth] 
        #LEFT
        if angle > 220 and angle < 300: 
            endPos = [x - depth, y] 

        
        for x, y in zip(range(currentPos[0], endPos[0]), range(currentPos[1], endPos[1])):
          if x == endPos[0] and y == endPos[1]: 
              self.explored[x, y] = 2 # To signify a wall
          else:
              self.explored[x, y] = 1
        
          
    def searchmode(self, image, mask):
        if self.search == False:
            print "entering searchmode"
        self.alreadyexplored()
        self.search = True
        self.seek = False
        self.found = False
        self.searchtimes +=1
        self.spinTimes += 1
        if self.spinTimes < 200:
            #print "spinning: " + str(self.spinTimes)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)
        else:
            #print "moving range total: " + str(np.nanmean(self.dataranges.ranges[260:380]))
            #print "moving range left : " + str(np.nanmean(self.dataranges.ranges[260:320]))
            #print "moving range right: " + str(np.nanmean(self.dataranges.ranges[320:380]))
            
            
            
            
            #print "min total: " + str(min(self.dataranges.ranges))
            #print "min right %s -- mean right %s -- combined %s : " % (str(min(self.dataranges.ranges[:320])), str(np.nanmean(self.dataranges.ranges[260:320])), self.right)             
            #print "min left %s -- mean left %s -- combined %s : " % (str(min(self.dataranges.ranges[320:])), str(np.nanmean(self.dataranges.ranges[320:380])), self.left) 
            if self.searchtimes > 600:
                #print "spinning"
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3
                self.cmd_vel_pub.publish(self.twist)
                if self.searchtimes > 1050:
                    self.searchtimes = 0
                    print "stopped spinning"
            elif min(self.dataranges.ranges) > 0.5:
                self.commandChanges = 0
                self.twist.linear.x = 0.5
                if np.nanmean(self.dataranges.ranges[260:380]) > 6.0:
                    
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if abs(self.right-self.left) < 0.2:
                        print "over auto move right"
                        self.twist.angular.z = -0.3
                    elif str(self.left) == "nan":
                        self.twist.angular.z = 0.3
                    elif str(self.right) == "nan":
                        self.twist.angular.z = -0.3
                    elif self.left > self.right:
                        print "over moving left"
                        self.twist.angular.z = 0.3
                    else:
                        print "over moving right"
                        self.twist.angular.z = -0.3
                    self.cmd_vel_pub.publish(self.twist)
            else:
                self.twist.linear.x = 0.0
                print str(self.commandChanges)
                if self.commandChanges > 15:
                    print "force move left"
                    if not self.command == "force move left":
                        self.command == "force move left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                elif abs(self.right-self.left) < 0.5:
                    print "auto move left"
                    if not self.command == "auto move left":
                        self.command == "auto move left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                elif str(self.left) == "nan":
                    print "moving left"
                    if not self.command == "moving left":
                        self.command == "moving left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                elif str(self.right) == "nan":
                    print "moving right"
                    if not self.command == "moving right":
                        self.command == "moving right"
                        self.commandChanges += 1
                    self.twist.angular.z = -0.3
                elif self.left > self.right:
                    print "moving left"
                    if not self.command == "moving left":
                        self.command == "moving left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                else:
                    print "moving right"
                    if not self.command == "moving right":
                        self.command == "moving right"
                        self.commandChanges += 1
                    self.twist.angular.z = -0.3
                self.cmd_vel_pub.publish(self.twist)
        
        cv2.waitKey(1)
        
    def seekmode(self, image, M):
        if self.seek == False:
            print "entering seekmode"
        self.seek = True
        self.search = False
        self.found = False
        h, w, d = image.shape
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        self.centrePointX = cx
        self.centrePointY = cy
        # make a circle of the centre and add it to the image
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
          
        err = cx - w/2
        print "min total: " + str(min(self.dataranges.ranges))
        if min(self.dataranges.ranges) < 0.5 or str(min(self.dataranges.ranges)) == "nan":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
        else:
            self.twist.linear.x = 0.5 # set speed 
            self.twist.angular.z = -float(err) / 100 # set turn based on the amount of error from angle of current turtlebot
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        cv2.waitKey(1)
    
    def foundmode(self, image, M):
        if self.found == False:        
            print "entering foundmode"
            if self.MR > self.MG and self.MR > self.MY and self.MR > self.MB:
                self.color = "red"
            elif self.MG > self.MR and self.MG > self.MY and self.MG > self.MB:
                self.color = "green"
            elif self.MB > self.MR and self.MB > self.MY and self.MB > self.MG:
                self.color = "blue"
            else:
                self.color = "yellow"
            print "found color: " + self.color
        self.found = True
        self.search = False
        self.seek = False
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.3
        self.cmd_vel_pub.publish(self.twist)
        
        #self.founditer += 1
        # END CONTROL
       # if self.depth < 0.5 and M['m00'] > 100000 and self.founditer > 10:
       #     print "stopping spin at iter: " + str(self.founditer)
        self.found = False
        print "removing color " + self.color + " from mask"
        self.colors.remove(self.color)
        print self.colors
        #self.founditer = 0
        cv2.waitKey(1)
                

rospy.init_node('follower')
follower = Follower()
rospy.spin()
