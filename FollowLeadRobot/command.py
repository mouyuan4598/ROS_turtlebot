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
#移动
class directionMove():
    def __init__(self,x,y):
	self.cmd_vel = rospy.Publisher('robot2/cmd_vel_mux/input/navi', Twist, queue_size=10)
	self.action(x,y)


    def action(self,x,y):
	move_cmd = Twist()
	move_cmd.angular.z = x
	move_cmd.linear.x = y
	self.cmd_vel.publish(move_cmd)

#控制
class Action():
    def __init__(self):
	self.coordinate_sub = rospy.Subscriber('/coordinate', Point, self.callback)

    def callback(self,data):
	x_axis = data.x
	y_axis = data.y
    #当机器人的位姿偏左或偏右，进行转动
	if y_axis < 300:
	    yy = 0.6
        elif y_axis < 370:
	    yy = 0.4
        else:
	    yy = 0

        #当机器人与前面机器人偏远的时候进行前进
        if x_axis > 370:
            xx = -0.2
        elif x_axis < 280:
            xx = 0.2
	else:
	    xx = 0
        rospy.loginfo("y_axis")
        #调用移动的函数
        directionMove(xx,yy)

def main():
    rospy.init_node("testing3")
    Action()
    rospy.spin()

    
        


if __name__ == "__main__":
    main()
