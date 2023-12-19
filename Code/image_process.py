import cv2
import numpy as np
import math
import numpy as np
from numpy import linalg as LA
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Accel
from tf2_ros import TransformBroadcaster

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time

self.subscription_imu = self.create_subscription(Imu, 'terrasentia/imu', self.callback_imu, 10)
def callback_imu(self,msg):
    #measurement vector = [p, q, r, fx, fy, fz, x, y, z, vx, vy, vz]
    # In practice, the IMU measurements should be filtered. In this coding exercise, we are just going to clip
    # the values of velocity and acceleration to keep them in physically possible intervals.
    self.measure[0] = np.clip(msg.angular_velocity.x,-5,5) #(-5,5)
    self.measure[1] = np.clip(msg.angular_velocity.y, -5, 5) #..(-5,5)
    self.measure[2] = np.clip(msg.angular_velocity.z, -5, 5) #..(-5,5)
    self.measure[3] = np.clip(msg.angular_velocity.y, -6, 6) #..(-6,6)
    self.measure[4] = None #..(-6,6)
    self.measure[5] = None #..(-16,-4) 

'''
#realtime
cap = cv2.VideoCapture(0)
while (cap.isOpened()):
    ret, frame = cap.read()


cap.release()
'''
def ImageFilter():
    #video
    source = np.float32([[400, 180], [800, 180], [400, 540], [800, 540]])
    dest = np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]])
    M = cv2.getPerspectiveTransform(source, dest)
    cap = cv2.VideoCapture('experiment.mp4')

    while(True):
        ret, frame = cap.read()
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low = np.array([26, 43, 46])
        high = np.array([34, 255, 255])
        extract = cv2.inRange(img_hsv, low, high)
        gaussian = cv2.GaussianBlur(extract, (7, 7), 0)
        edges = cv2.Canny(gaussian, 50, 150, apertureSize=3)
        edges = edges[360:720, :]
        edge_pixels = np.argwhere(edges > 0)
        error = 0
        count = 0
        # for edge_pixel in edge_pixels:
        #     error += (edge_pixel[1]-640)
        #     count+=1
            



        perspective_tranform = cv2.warpPerspective(gaussian, M, (1280, 720))
        cv2.imshow('lane', edges)
        cv2.waitKey(0)
        break
    
