#!/usr/bin/env python

from pickle import NONE
from re import A
#from types import NoneType
import rospy
from sensor_msgs.msg import Image,LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import cv2
from cv_bridge import CvBridge, CvBridgeError
import math


import numpy as np
from numpy import linalg as LA

import glob
from skimage import morphology
import time
import matplotlib.pyplot as plt

class Motion(object):
    def __init__(self):
        print('initialze img class')
        #publish to
        ctr_topic="/vesc/low_level/ackermann_cmd_mux/input/navigation" #AckermannDrive

        self.controlPub = rospy.Publisher(ctr_topic, AckermannDrive, queue_size = 1)
        
    def cycle(self):
        i=0    
        while not rospy.is_shutdown():

            target_steering = 0.6
            target_velocity=  1
 #           print('steering_ang  ',target_steering)
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.speed = target_velocity
            newAckermannCmd.steering_angle = target_steering

            self.controlPub.publish(newAckermannCmd)
            self.rate.sleep()
        
def main():
    rospy.init_node('controller', anonymous=True)
    model=Motion()
    '''
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    '''
    try:
        model.cycle()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
