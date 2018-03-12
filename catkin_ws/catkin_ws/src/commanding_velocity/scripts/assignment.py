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



class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        #cv2.namedWindow("depth", 1)
        cv2.namedWindow("mask", 1)        
        self.image_sub = rospy.Subscriber("turtlebot/camera/rgb/image_raw",
                                          Image, self.callback)
        self.image_depth = rospy.Subscriber("turtlebot/scan", LaserScan, self.depthcallback)          
        self.cmd_vel_pub = rospy.Publisher("turtlebot/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.centrePointX = 0
        self.centrePointY = 0
        self.depth = 100000000
        self.dataranges = 0
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
        self.spinTimes = 0
        self.mask = self.maskGreen + self.maskRed + self.maskYellow
        self.colors = ["red", "green", "yellow"]
        
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
        
        self.maskGreen  = cv2.inRange(hsv, lower_green,  upper_green)
        self.maskRed    = cv2.inRange(hsv, lower_red,    upper_red)
        self.maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        self.MG = cv2.moments(self.maskGreen)['m00']
        self.MR = cv2.moments(self.maskRed)['m00']
        self.MY = cv2.moments(self.maskYellow)['m00']
        
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
            
        h, w, d = image.shape
    
        # calculate the area of the values that are within the mask
        M = cv2.moments(self.mask)
        
        if M['m00'] > 100000:
            if self.depth > 0.6:
                self.seekmode(image, M)
            else:
                self.foundmode(image, M)
        else:
            self.searchmode(image, self.mask)
          
    def searchmode(self, image, mask):
        if self.search == False:
            print "entering searchmode"
        self.search = True
        self.seek = False
        self.found = False
        self.searchtimes +=1
        self.spinTimes += 1
        if self.spinTimes < 200:
            print "spinning: " + str(self.spinTimes)
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)
        else:
            #print "moving range total: " + str(np.nanmean(self.dataranges.ranges[260:380]))
            #print "moving range left : " + str(np.nanmean(self.dataranges.ranges[260:320]))
            #print "moving range right: " + str(np.nanmean(self.dataranges.ranges[320:380]))
            
            right  = min(self.dataranges.ranges[:320]) + np.nanmean(self.dataranges.ranges[260:380])
            left = min(self.dataranges.ranges[320:]) + np.nanmean(self.dataranges.ranges[320:380])
            
            
            print "min total: " + str(min(self.dataranges.ranges))
            print "min right %s -- mean right %s -- combined %s : " % (str(min(self.dataranges.ranges[:320])), str(np.nanmean(self.dataranges.ranges[260:320])), right)             
            print "min left %s -- mean left %s -- combined %s : " % (str(min(self.dataranges.ranges[320:])), str(np.nanmean(self.dataranges.ranges[320:380])), left) 
            
            
            if self.searchtimes > 600:
                print "spinning"
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.3
                self.cmd_vel_pub.publish(self.twist)
                if self.searchtimes > 1050:
                    self.searchtimes = 0
                    print "stopped spinning"
            elif min(self.dataranges.ranges) > 0.7:
                self.commandChanges = 0
                self.twist.linear.x = 0.2
                if np.nanmean(self.dataranges.ranges[260:380]) > 4.0:
                    
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if abs(right-left) < 0.5:
                        print "over auto move left"
                        self.twist.angular.z = 0.3
                    elif str(left) == "nan":
                        self.twist.angular.z = 0.3
                    elif str(right) == "nan":
                        self.twist.angular.z = -0.3
                    elif left > right:
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
                elif abs(right-left) < 0.5:
                    print "auto move left"
                    if not self.command == "auto move left":
                        self.command == "auto move left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                elif str(left) == "nan":
                    print "moving left"
                    if not self.command == "moving left":
                        self.command == "moving left"
                        self.commandChanges += 1
                    self.twist.angular.z = 0.3
                elif str(right) == "nan":
                    print "moving right"
                    if not self.command == "moving right":
                        self.command == "moving right"
                        self.commandChanges += 1
                    self.twist.angular.z = -0.3
                elif left > right:
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
        cv2.imshow("window", image)
        cv2.imshow("mask", self.mask)
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
        self.twist.linear.x = 0.1 # set speed 
        self.twist.angular.z = -float(err) / 100 # set turn based on the amount of error from angle of current turtlebot
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        cv2.imshow("window", image)
        cv2.imshow("mask", self.mask)
        cv2.waitKey(1)
    
    def foundmode(self, image, M):
        if self.found == False:        
            print "entering foundmode"
            if self.MR > self.MG and self.MR > self.MY:
                self.color = "red"
            elif self.MG > self.MR and self.MG > self.MY:
                self.color = "green"
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
        cv2.imshow("window", image)
        cv2.imshow("mask", self.mask)
        cv2.waitKey(1)
                

rospy.init_node('follower')
follower = Follower()
rospy.spin()
