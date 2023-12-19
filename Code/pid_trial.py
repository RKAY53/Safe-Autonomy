#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray


from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

"""
*Sensing
/scan: this topic maintains the LaserScan messages published by the LiDAR.

/odom: this topic maintains the Odometry messages published by the VESC.

/sensors/imu/raw: this topic maintains the Imu messages published by the VESC.

/sensors/core: this topic maintains the VescStateStamped messages published by the VESC on telemetry data.

*Control
/drive: this topic is listened to by the VESC, needs AckermannDriveStamped messages. 
The speed and steering_angle fields in the drive field of these messages are used to command desired steering and velocity to the car.
"""

#https://github.com/mit-racecar/vesc/blob/master/vesc_msgs/msg/VescState.msg

from cv_bridge import CvBridge, CvBridgeError
#
#
#/camera/camera/color/image_raw
class lanenet_detector():
    def __init__(self):
        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)





class perception(object):
    def __init__(self):

        self.rate = rospy.Rate(30)


        #pf_topic = rospy.get_param('pose_topic')
        #scan_topic = rospy.get_param('scan_topic')

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.ctrl_pub  = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)

class PID():
    #
    ##https://github.com/Jmack66/PIDtest/blob/master/PID_Video.py
	def __init__(self,KP,KI,KD):
		self.kp = KP
		self.ki = KI
		self.kd = KD
		self.target = 0
		self.error_prev = 0
		self.integral_error = 0
		#max_ctr=45 degree
		self.max_ctr = 0.785
    def compute(self,measured,dt):
		error=measured
		derivative_error = (error - self.error_prev) / dt
		self.integral_error += error * dt 
		output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error 
		self.error_prev = error
		if output > self.max_ctr:
			output = self.max_ctr
		elif output < -self.max_ctr: 
			output = -self.max_ctr
		return output

if __name__ == '__main__':
	while True:
		'''
		-filter the line
		-projective transform
		-line fitting=>get the equation
		-get 


		'''
		state=getModelState()
		#filtering
		#linefitting
		#control function()
		#    steering=PID
		#    v=sth
		#	 publish
