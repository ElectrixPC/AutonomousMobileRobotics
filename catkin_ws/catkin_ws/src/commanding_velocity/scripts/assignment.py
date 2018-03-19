# -*- coding: utf-8 -*-
"""
ALL1346037 - CMP3103M Assessment Item 1

@author: James Allington-Kay
"""

#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi, cos, sin, degrees, floor

class Assignment:

    def __init__(self):
        ''' Function for initialising values and setting up subscribers '''
        # Init cvbridge for use in images
        self.bridge = cv_bridge.CvBridge()
        # Define named windows for displaying
        cv2.namedWindow("window", 1)
        cv2.namedWindow("explored", 1)
        cv2.namedWindow("mask", 1)        
        # Set up subscribers and publishers
        self.image_sub = rospy.Subscriber("turtlebot/camera/rgb/image_raw", Image, self.imageCallback)
        self.image_depth = rospy.Subscriber("turtlebot/scan", LaserScan, self.depthCallback)
        self.image_odom = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.exploreCallback)         
        self.cmd_vel_pub = rospy.Publisher("turtlebot/cmd_vel", Twist, queue_size=1)
        # Initialise twist value for controling bot        
        self.twist = Twist()
        # Init tall other values for future use
        self.centrePointX = 0
        self.centrePointY = 0
        self.depth = 0
        self.dataranges = 0
        self.firstDepthTime = True
        self.firstTime = True
        self.explored = np.zeros((1500, 1500))
        self.found = False
        self.search = False
        self.commandChanges = 0
        self.command = ""
        self.seek = False
        # Init colors and set up color arrays     
        self.maskGreen = 0
        self.maskRed = 0
        self.maskYellow = 0
        self.maskBlue = 0
        self.spinTimes = 0
        self.mask = self.maskGreen + self.maskRed + self.maskYellow + self.maskBlue
        self.colors = ["red", "green", "yellow", "blue"] 
        
        
    def depthCallback(self, data):
        ''' Function for receiving the depth values and assigning them to global values '''
        
        # Assign whole data to dataranges for futher use
        self.dataranges = data
        
        if not self.centrePointX == 0: # Check if a centerpoint has been set
            if str(self.depth) == "nan": # Check if the depth is super close or super far away
                self.depth = 10
            else:
                self.depth = data.ranges[self.centrePointX]
        else:
            if str(self.depth) == "nan":
                self.depth = 10
            else:
                self.depth = data.ranges[320] #If no centerpoint has been set, choose middle value
                
                
        # Calculate minumum value from left and right side of center depth, plus the mean of the left and right
        # Resultant values are not swayed by large distances, and is sensitive to close values
        self.right  = min(self.dataranges.ranges[:320]) + np.nanmean(self.dataranges.ranges[260:380]) 
        self.left   = min(self.dataranges.ranges[320:]) + np.nanmean(self.dataranges.ranges[320:380])
        # Wait, and set the first time variable to false
        self.firstDepthTime = False
        cv2.waitKey(1)


    def imageCallback(self, data):
        ''' Function for receiving the image values and controlling the overall 
            application based on the colors in the image '''
            
        # Collect data value and convert it to an image in HSV color format
        image = self.bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Set upper and lower bounds for the colors in hsv format
        lower_green = np.array([ 45, 100,  50])
        upper_green = np.array([ 75, 255, 255])
        lower_red = np.array([ 0, 200,  100])
        upper_red = np.array([ 5, 255, 255])
        lower_yellow = np.array([ 20, 200,  100])
        upper_yellow = np.array([ 50, 255, 195])
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([150, 255, 255])
        
        # Set the masks based on the hsv image
        self.maskGreen  = cv2.inRange(hsv, lower_green,  upper_green)
        self.maskRed    = cv2.inRange(hsv, lower_red,    upper_red)
        self.maskYellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        self.maskBlue   = cv2.inRange(hsv, lower_blue,   upper_blue)
        # Get the area of color in the mask for each color
        self.MG = cv2.moments(self.maskGreen)['m00']
        self.MR = cv2.moments(self.maskRed)['m00']
        self.MY = cv2.moments(self.maskYellow)['m00']
        self.MB = cv2.moments(self.maskBlue)['m00']        
        # Reset mask before checking colors
        self.mask = None
        # Check colors that haven't been removed yet and add to the mask
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
    
        # Get the area of the colors remaining in the mask
        M = cv2.moments(self.mask)
        # Display the image and the mask
        cv2.imshow("window", image)
        cv2.imshow("mask", self.mask)
        # Check if it isnt the first iteration
        if not self.firstDepthTime:
            # Check if the area of the mask is over a certain threshold (point found and seeking)
            if M['m00'] > 100000:
                # Check if the depth is less than 75 cm and the area is over a certain threshold 
                if self.depth < 0.75 and M['m00'] > 10000000:
                     # If it is, enter found mode
                     self.foundmode(image, M)
                else:
                     # If iti isnt, enter seek mode as you are too far away
                     self.seekmode(image, M)
            else:
                # If it can't see anything, go into searchmode
                self.searchmode(image, self.mask)
        # Wait
        cv2.waitKey(1)

    def odom_orientation(self, q):
        ''' function for calculating value in degrees from quaternion'''
        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
        return y * 180 / pi
        
    def exploreCallback(self, data):
        ''' function for getting the current position and angle, and calculating
            the explored area of the map for use in the searchmode'''
        
        # Get X and Y values, and multiply them by 50 to increase size on generated map
        x = int(data.pose.pose.position.x * 50)
        y = int(data.pose.pose.position.y * 50)       
        # Init position in center of map at the start (total size is 1500x1500)
        if self.firstTime:
            self.diffToStart = [x-750, y-750]
            self.firstTime = False
        # Set global values for x and y for future use by calculating the difference to start
        self.x = x  - self.diffToStart[0]
        self.y = y  - self.diffToStart[1]
        # set glboal current position
        self.currentPos = [self.x, self.y]
        # Calculate the minimum, center and max angle for the robot by getting the data from the callback
        minangle = int(self.odom_orientation(data.pose.pose.orientation) + degrees(self.dataranges.angle_min))
        self.angle = int(self.odom_orientation(data.pose.pose.orientation))
        maxangle = int(self.odom_orientation(data.pose.pose.orientation) + degrees(self.dataranges.angle_max))
        
        # Check if the depth is super close or super high
        if str(self.depth) == "nan":
            self.depth = 0 # Assume that the depth is zero, so it doesn't interfere with the map
        depth = int(abs(self.depth) *10) # Multiply the depth to fit in the map
        # get the size of the depth array
        size = len(self.dataranges.ranges)
        # calculate an angle for each of the values in the depth array
        iterable = np.linspace(minangle, maxangle, size)
        # Loop through to size of the depth array, iterating by 10 each time to speed things up
        for i in range(0, size-1, 10):
            angle = iterable[i] # get angle
            basedepth = self.dataranges.ranges[i]
            if str(basedepth) == "nan":
                basedepth = 10
            
            depth = int(abs(basedepth) *10) # get depth of individual point
            if not str(depth) == "nan": # check if depth isnt nan
                # Use trigonometry to calculate the end position based on the depth and the angle, and round them down to ints
                endPos = [int(floor(self.y + depth*sin(angle))), int(floor(self.x + depth*cos(angle)))]
                # set that value to the explored array to be white
                self.explored[endPos[0]][endPos[1]] = 255
        # create a numpy array from the explored values
        im = np.array(self.explored, dtype = np.uint8)
        # convert the array to an image
        threshed = cv2.adaptiveThreshold(im, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
        # flip the image to show on the display properly
        self.threshed = cv2.flip(threshed, 0)
        cv2.imshow("explored", self.threshed)
        
    def commandmoveleft(self, name, recordChanges = False):
        '''Function for moving left'''
        # Record the number of changes to see whether it is erroring
        if recordChanges:
            if not self.command == name:
                self.command == name
                self.commandChanges += 1
        # Set angular turn and publish
        self.twist.angular.z = 0.4
        self.cmd_vel_pub.publish(self.twist)
            
    def commandmoveright(self, name, recordChanges = False):
        '''Function for moving right'''
        # Record the number of changes to see whether it is erroring
        if recordChanges:
            if not self.command == name:
                self.command == name
                self.commandChanges += 1
        # Set angular turn and publish
        self.twist.angular.z = -0.4
        self.cmd_vel_pub.publish(self.twist)
        
    def autocontrolbot(self, seekMode = False):
        ''' function for automatically controlling the robot from hitting things
            TO BE USED ONLY WHEN THERE ISNT AN ALTERNATIVE'''
        # set forward motion to zero
        self.twist.linear.x = 0.0
        # if more than 50 changes in command have been made without moving forward, force it to move left
        if self.commandChanges > 50:
            self.commandmoveleft("force move left", True)
        # Check if the difference between right and left is very small
        # if it is reset the values of left and right to be the maximum so it changes bias, and calls itself again
        elif abs(self.right-self.left) < 0.5:
            if seekMode:
                self.right  = max(self.dataranges.ranges[:320]) + np.nanmean(self.dataranges.ranges[260:380])
                self.left   = max(self.dataranges.ranges[320:]) + np.nanmean(self.dataranges.ranges[320:380])
                self.autocontrolbot(seekMode = True)
            else:
                self.commandmoveleft("auto move left", True)
        elif str(self.left) == "nan": # If left is really close, move left
            self.commandmoveleft("moving left", True)
        elif str(self.right) == "nan": # If right is really close, move right
            self.commandmoveright("moving right", True)
        elif self.left > self.right: # If left is much higher than right, move left
            self.commandmoveleft("moving left", True)
        else: # Move right if everything else doesn't work
            self.commandmoveright("moving right", True)
        self.cmd_vel_pub.publish(self.twist)
        
    def searchmode(self, image, mask):
        ''' Mode for searching the area'''
        if self.search == False:
            print "entering searchmode"
        # Reset mode variables
        self.search = True
        self.seek = False
        self.found = False
        # If it hasn't spun yet in the algorithm, spin        
        self.spinTimes += 1        
        if self.spinTimes < 200:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
            self.cmd_vel_pub.publish(self.twist)
        else:
            # get average explored value in the area of the robot, calculated by the exploreCallback function
            values = []
            for i, j in zip(range(self.x - 100, self.x + 100), range(self.y - 100, self.y + 100)):
                values.append(self.explored[i, j]) 
            avgexporation = np.mean(values)
            
            # Check if the minimum value is over 75cm
            if min(self.dataranges.ranges) > 0.75:
                # reset the commandChanges from search
                self.commandChanges = 0
                # set speed of robot
                self.twist.linear.x = 0.5
                # Check if the center area of depth is over 4m, and the exploration is under 0.5 
                if np.nanmean(self.dataranges.ranges[260:380]) > 4.0 and avgexporation < 0.5:
                    # just move forward
                    self.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    # Check if the difference between left and right is less than 0.2
                    if abs(self.right-self.left) < 0.2:
                        self.commandmoveright("over auto move right") # If it is, automatically move right
                    elif str(self.left) == "nan":
                        self.commandmoveleft("max move left") # If left is nan, means it is really far away, move left
                    elif str(self.right) == "nan":
                        self.commandmoveright("max move right") # If right is nan, means it is really far away too, move right
                    elif self.left > self.right:
                        self.commandmoveleft("over moving left") # If left is a lot more than right, move left
                    else:
                        self.commandmoveright("over moving right") # else move right
            else:
                self.autocontrolbot() # If the distance is under 4m and the area has already been explored, automatically control the robot
        # Wait
        cv2.waitKey(1)
        
    def seekmode(self, image, M):
        ''' Function for seeking the post '''
        if self.seek == False:
            print "entering seekmode"
        # Set the mode variables
        self.seek = True
        self.search = False
        self.found = False
        # Get the width of the image
        _, w, _ = image.shape
        # Get the centerpoint of the masked area that has been detected
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        # set those values to the X and Y variables
        self.centrePointX = cx 
        self.centrePointY = cy
        # make a circle of the centre and add it to the image
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        # calculate the error (the angle of error to the center cx value)
        err = cx - w/2
        # If the robot can't get to the post, go to the autocontrolbot mode
        if min(self.dataranges.ranges) < 0.75 or str(min(self.dataranges.ranges)) == "nan":
            self.autocontrolbot(seekMode = True)
        else:
            # direct the robot in the direction of the error value
            self.commandChanges = 0
            self.twist.linear.x = 0.5 # set speed 
            self.twist.angular.z = -float(err) / 100 # set turn based on the amount of error from angle of current turtlebot
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        cv2.waitKey(1)
    
    def foundmode(self, image, M):
        ''' Function for finding the post at the correct distance and area '''
        if self.found == False:        
            print "entering foundmode"
        self.found = True
        self.search = False
        self.seek = False
        # Work out which color has been found by looking at the masks
        if self.MR > self.MG and self.MR > self.MY and self.MR > self.MB:
            self.color = "red"
        elif self.MG > self.MR and self.MG > self.MY and self.MG > self.MB:
            self.color = "green"
        elif self.MB > self.MR and self.MB > self.MY and self.MB > self.MG:
            self.color = "blue"
        else:
            self.color = "yellow"
        
        print "found color: " + self.color
        # Move forward after finding
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5
        self.cmd_vel_pub.publish(self.twist)
        # Reset found flag
        self.found = False
        # Remove color from the array, so it won't find it again
        print "distance: " + str(min(self.dataranges.ranges))
        print "removing color " + self.color + " from mask"
        self.colors.remove(self.color)
        print "remaining: " + ' '.join(str(e) for e in self.colors)
        # If found everything, exit script
        if len(self.colors) == 0:
            print("COMPLETED")
            quit()
        
        cv2.waitKey(1)
                
# Start application
rospy.init_node('assignment')
assignment = Assignment()
rospy.spin()
