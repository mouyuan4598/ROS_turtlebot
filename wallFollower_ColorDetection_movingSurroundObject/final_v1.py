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
              }

#定义全局变量
#由于电脑的配置不够高，导致在运行时opencv的处理会严重延时，最终会导致当摄像头已经对到了颜色块，可是由于处理不及，
#摄像头转向了，导致无法读取颜色
#done的意义是当opencv成功到处经过处理的画面时返回
#deteced是当读取到想要的颜色（蓝色）会返回True
sensor_ = [] 
detected_ = False
done_ = False


class Image_converter:
    def __init__(self):
	self.detected = False
	self.bridge = CvBridge()
	rospy.loginfo("initt")
	self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,self.callback)
	self.image_pub = rospy.Publisher('table_detect_test',Image,queue_size = 10)
    	# Allow up to one second to connection
 
    def callback(self,data):
		
		# Convert image to OpenCV format，
		try:
		    cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
		    pass


		detect_image = self.detect_table(cv_image)
		try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, "bgr8"))
			
        	except CvBridgeError as e:
            	    pass
			

#以上沿用助教提供的代码

    def detect_table(self,image):

	global detected_
	global done_
	g_image = cv2.GaussianBlur(image, (5, 5), 0)		
	hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)          
        erode_hsv = cv2.erode(hsv, None, iterations=2)
        color = ['red', 'blue', 'green']       
	#修改部分，循环三种颜色         
        for i in color:
            inRange_hsv = cv2.inRange(erode_hsv, color_dist[i]['Lower'], color_dist[i]['Upper'])
            cnts = cv2.findContours(inRange_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            #直到成功侦测到任意一种颜色
	    if cnts:
                c = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                cv2.drawContours(image, [np.int0(box)], -1, (0, 255, 255), 2)
	  	#如果是想要的蓝色，detected返回true，想要什么颜色，变量在这里换
		if i == 'blue':
		    self.image_sub.unregister()
		    self.detected = True
		    
		    detected_ = True
                #rospy.loginfo(i)
		done_ = True    
		return image
	#当成功完成一次侦测，done返回true
	done_ = True




class sensorLaser():
    def __init__(self):
	rospy.loginfo("init")	
	self.sensor = []
	self.laser_sub = rospy.Subscriber('/turtlebot/laser/scan',LaserScan,self.output)
    def output(self,array):
	global sensor_
	'''rospy.loginfo(len(array.ranges))
	rospy.loginfo(array.ranges[0])
        rospy.loginfo(array.ranges[90])
	rospy.loginfo(array.ranges[360])
	rospy.loginfo(array.ranges[630])
	rospy.loginfo(array.ranges[719]) 	     	
	rospy.loginfo("endddd") 
	'''
	#只利用激光雷达的五个点 左前右
	self.sensor = [array.ranges[0],array.ranges[65],array.ranges[320],array.ranges[574],array.ranges[639]]
	sensor_ = self.sensor

class directionMove():
    def __init__(self,sensor):
	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	self.action(sensor)


    def action(self,sensor):
	margin = 0.07
	#当中间激光侦测小于一定距离
	if sensor[1] < 0.8 and sensor[2] < 1.0:
	    move_cmd = Twist()
	    move_cmd.angular.z = 1.5
	    self.cmd_vel.publish(move_cmd)	
	    rospy.loginfo("!!!!!!!")
	#当右边激光不再贴墙
	elif sensor[1] > 0.65 or sensor[0] >0.8 or (sensor[0]> sensor[1] and sensor[0] >0.75):
	    move_cmd = Twist()
	    move_cmd.angular.z = -0.25
	    move_cmd.linear.x = 0.16
	    self.cmd_vel.publish(move_cmd)
	    rospy.loginfo("?????")
	#当机器人距离墙壁的距离是满意的    
	elif sensor[0] > 0.5 -margin and sensor[0] < 0.5 + margin:
	     
	    if sensor[1] - sensor[0] > -margin and sensor[1] - sensor[0] < margin:		
		move_cmd = Twist()
		move_cmd.linear.x = 0.5
		self.cmd_vel.publish(move_cmd)
		rospy.loginfo("FORWARD!!!")
	#以下两个条件是让机器人与墙壁平行的调节	
            elif sensor[1] - sensor[0] < -margin:
		move_cmd = Twist()
		move_cmd.angular.z = +0.1
		move_cmd.linear.x = 0.2
		self.cmd_vel.publish(move_cmd)
		rospy.loginfo("left!!!")
	    elif sensor[1] - sensor[0] > margin:
		move_cmd = Twist()
		move_cmd.angular.z = -0.1    
		move_cmd.linear.x = 0.2
		self.cmd_vel.publish(move_cmd)
 	        rospy.loginfo("right!!!")
	#如果机器人距离墙壁太靠近或者太远
	else:
	    
	    if sensor[0] < 0.5 - margin:
	        move_cmd = Twist()

		if sensor[0] < 0.3:
	            move_cmd.angular.z = 0.15
		    move_cmd.linear.x = 0.12
		else:
		    move_cmd.angular.z = 0.05		
		    move_cmd.linear.x = 0.15
	        self.cmd_vel.publish(move_cmd)
	        rospy.loginfo("LEFTaaa!!!")
	    elif sensor[0] > 0.5 + margin:
	        move_cmd = Twist()	
	        if sensor[0] > 0.8:
	            move_cmd.angular.z = -0.15
		    move_cmd.linear.x = 0.12
		else:
		    move_cmd.angular.z = -0.05
		    move_cmd.linear.x = 0.15
	        self.cmd_vel.publish(move_cmd)
	        rospy.loginfo("RIGHTaaa!!!") 

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat,i):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'][i], pos['y'][i], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'][i], quat['r4'][i]))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def main():
    rospy.init_node("testing")
    r = rospy.Rate(10)
    #当还没有侦测到颜色，不断地循环进行跟墙
    while not detected_:
	global done_
	

        Image_converter()
        rospy.sleep(1)
	#opencv 处理完毕
        if done_:
	#进行跟墙
            sensorLaser()
	    if sensor_: 
	        rospy.loginfo(sensor_)
	        directionMove(sensor_)
		done_ = False
	r.sleep()
	#侦测到颜色后前往目标房间
    try:
        navigator = GoToPose()
        position = {'x': [13.84], 'y' :[-3.35]}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : [0.9597], 'r4' : [0.28]}
        for i in range(1):
	
            rospy.loginfo("Go to (%s, %s) pose", position['x'][i], position['y'][i])
            success = navigator.goto(position, quaternion,i)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

#以跟墙的方式让机器人绕着目标颜色的盒子转
    r = rospy.Rate(100)
    for i in range(5000):
	sensorLaser()
	if sensor_:
	    directionMove(sensor_)
	    rospy.loginfo(sensor_)
	    r.sleep()
    
#结束后走出房间
    try:
        navigator = GoToPose()
        position = {'x': [16.08], 'y' :[-5.76]}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : [-0.365], 'r4' : [0.93]}
        for i in range(1):
	
            rospy.loginfo("Go to (%s, %s) pose", position['x'][i], position['y'][i])
            success = navigator.goto(position, quaternion,i)

            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")    

if __name__ == "__main__":
    main()
