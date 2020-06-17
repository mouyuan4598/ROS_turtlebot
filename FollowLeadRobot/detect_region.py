#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np 
import math
from geometry_msgs.msg import Pose,Point, Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
	      'black': {'Lower': np.array([0,0,0]), 'Upper': np.array([50,36,33])},
              }
x_axis = 0
y_axis = 0 
done_ = False
class directionMove():
    def __init__(self,x,y):
	self.cmd_vel = rospy.Publisher('robot2/cmd_vel_mux/input/navi', Twist, queue_size=3)
	self.action(x,y)

    def action(self,x,y):
	move_cmd = Twist()
	move_cmd.angular.z = x
	move_cmd.linear.x = y
	self.cmd_vel.publish(move_cmd)

class Image_converter:
    def __init__(self):
	self.detected = False
	self.bridge = CvBridge()
	rospy.loginfo("initt")
	self.image_sub = rospy.Subscriber('robot2/camera/rgb/image_raw',Image,self.callback)
	
 
    def callback(self,data):
		
		# Convert image to OpenCV format，
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
		    pass


		detect_image = self.detect_table(cv_image)
		try:
		    image_pub = rospy.Publisher('table_detect_test',Image,queue_size = 10)
                    image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, "bgr8"))
			
        	except CvBridgeError as e:
            	    pass
			

#以上沿用助教提供的代码

    def detect_table(self,image):

	global x_axis
	global y_axis
	global done_
	g_image = cv2.GaussianBlur(image, (5, 5), 0)		
	hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)          
        erode_hsv = cv2.erode(hsv, None, iterations=2)     
	#修改部分，循环三种颜色    
	inRange_hsv = cv2.inRange(erode_hsv, color_dist['black']['Lower'], color_dist['black']['Upper'])
        cnts = cv2.findContours(inRange_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	       
	#当成功侦测到颜色
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(c)
	    x_axis = rect[0][0]
	    y_axis = rect[0][1]
	    coordinate = Point()
            coordinate.x = x_axis
 	    coordinate.y = y_axis
   	    coordinate_pub = rospy.Publisher('coordinate',Point,queue_size = 10)
   	    coordinate_pub.publish(coordinate)
   	    rospy.loginfo("published")
		   #publish 感性区域的坐标
            box = cv2.boxPoints(rect)
            cv2.drawContours(image, [np.int0(box)], -1, (0, 255, 255), 2)
	    done_=True
	    return image
	done_=True
	return erode_hsv

def main():
    rospy.init_node("testing2")
        
    Image_converter()

    rospy.spin()



if __name__ == "__main__":
    main()
