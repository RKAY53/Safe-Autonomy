#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image,LaserScan
import cv2
#from cv_bridge import CvBridge, CvBridgeError
import math

import numpy as np
from numpy import linalg as LA

import glob
from skimage import morphology
import time
import matplotlib.pyplot as plt


def main():
    rospy.init_node('lidar', anonymous=True)
    lidar_topic = "/scan"
    image_topic = "/D435I/color/image_raw"  # Adjust to match the topic for your camera
    #d_image_topic = "/D435I/depth/image_rect_raw"  # Adjust to match the topic for your camera

    rospy.Subscriber(lidar_topic, LaserScan, lidar_callback)
    # Keep the program alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
def lidar_callback(msg):
    print(msg.ranges)

if __name__ == '__main__':
    main()