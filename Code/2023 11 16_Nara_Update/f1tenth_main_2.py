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

class imageProcessing():
    def __init__(self):
        self.bridge = CvBridge()
        self.x=None
        self.y=None
        self.binary_image=None
        self.coeff=None
        self.raw_img=None
        image_topic = "/D435I/color/image_raw"  # Adjust to match the topic for your camera
        #d_image_topic = "/D435I/depth/image_rect_raw"  # depth camera
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        #self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        #self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        #self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        #self.left_line = Line(n=5)
        #self.right_line = Line(n=5)
        #elf.detected = False
        #self.hist = True

    def image_callback(self,ros_image):
        try:
            # Convert the ROS Image message to a format OpenCV understands
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            #z16
        
        except CvBridgeError as e:
            print(e)
        #raw_img = cv_image.copy()
        self.raw_img=cv_image.copy()
        # cv2.imshow('tes',raw_img)
        # cv2.waitKey(0)
        #self.x,self.y,self.binary_image,self.coeff = self.color_thresh(raw_img)
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_binary_image(self):
        return self.binary_image
    def get_raw_image(self):
        return self.raw_img
    def get_coeff(self):
        return self.coeff
    def d_image_callback(self,ros_image):
        try:
            # Convert the ROS Image message to a format OpenCV understands
            cv_image = self.bridge.imgmsg_to_cv2(ros_image,desired_encoding='passthrough')
            #z16
        except CvBridgeError as e:
            print(e)

        # Use the current time as the file name
        file_name = "image_cal{}.jpg".format(time.strftime("%Y%m%d-%H%M%S"))
        # Save the image using OpenCV
        # print(cv_image.shape)
        # cv_image = ImageFilter(cv_image)
    #    print(cv_image)
        cv2.imshow('show2', cv_image)
        cv2.waitKey(0)
        #print('shape   ',cv_image.shape)




KP=0.0001
KI=0.1
KD=0.1
class PID():

    ##htps://github.com/Jmack66/PIDtest/blob/master/PID_Video.py

    def __init__(self,KP,KI,KD):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.target = 0
        self.error_prev = 0
        self.integral_error = 0
		#max_ctr=45 degree
        self.max_ctr = 0.5
        self.ctr=[[0,0]]
        #self.y_ahead=240 #in img pixel coordinate
        self.prev_time = time.time()
        self.cur_time = time.time()

    def compute(self,error):
        self.cur_time = time.time()
        dt=self.cur_time-self.prev_time
        #error,_,_,_=error_compute(self.y_ahead,self.line_fit)
        derivative_error = (error - self.error_prev) / dt
        self.integral_error += error * dt 
        output = self.kp*error + self.ki*self.integral_error + self.kd*derivative_error 
        #self.ctr.append=self.ctr.append.append(self.ctr.append, [dt,output])
        self.ctr.append([(self.ctr[-1][0]+dt),output])
        #self.ctr.append(output)
        self.error_prev = error
        self.prev_time=self.cur_time
        if output > self.max_ctr:
            output = self.max_ctr
        elif output < -self.max_ctr: 
            output = -self.max_ctr       
        return output
    def get_ctr(self):
        return self.ctr

class Motion(object):
    def __init__(self):
        self.rate = rospy.Rate(30)
        #print('initialze img class')
        #self.img_processing=imageProcessing()
        #publish to
        ctr_topic="/vesc/low_level/ackermann_cmd_mux/input/navigation" #AckermannDrive


        self.ctrl_pub  = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.drive.speed     = 1.0 # m/s, reference speed








        #ctr_topic="/ackermann_cmd" Ackerman
        # nDrive
        self.coeff=None
        self.prev_steering=0
        #print('publish control topic')

        #self.controlPub = rospy.Publisher(ctr_topic, AckermannDrive, queue_size = 1)
        self.pid = PID(KP,KI,KD)

        self.bridge = CvBridge()
        self.x=None
        self.y=None
        self.binary_image=None
        self.coeff=None
        self.raw_img=None
        image_topic = "/D435I/color/image_raw"  # Adjust to match the topic for your camera
        #d_image_topic = "/D435I/depth/image_rect_raw"  # depth camera
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        #self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        #self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        #self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        #self.left_line = Line(n=5)
        #self.right_line = Line(n=5)
        #elf.detected = False
        #self.hist = True

    def image_callback(self,ros_image):
        try:
            # Convert the ROS Image message to a format OpenCV understands
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            #z16
        
        except CvBridgeError as e:
            print(e)
        #raw_img = cv_image.copy()
        self.raw_img=cv_image.copy()

    def count_pix(self,h,perspective_transform):
        row_arr=perspective_transform[h]
        number_counts={}
        count_n=0
        s_pt=0
        for i in range(len(row_arr)):
            if row_arr[i]==0:
                if count_n!=0:
                    if count_n>=50:
                        number_counts[(s_pt,i-1,h)]=count_n
                count_n=0
            else:
                count_n+=1
                if count_n==1:
                    s_pt=i
        return number_counts

    def color_thresh(self,file):
        #img = cv2.imread(file)
        img=file
        height, width = img.shape[:2]
        hsl_img=cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        hsl_img_s=hsl_img[:,:,2]

        source = np.float32([
        [60, 400],
        [580,400],
        [0, 480],
        [640, 480]
            ])
        dest = np.float32([
            [0, 0],
            [640, 0],
            [0, 480],
            [640, 480]
        ])
    
        M = cv2.getPerspectiveTransform(source, dest)
        perspective_transform = cv2.warpPerspective(hsl_img_s, M, (width, height))
        _,perspective_transform = cv2.threshold(perspective_transform,80,255,cv2.THRESH_BINARY)

        h_1=height*30//100
        h_2=height*30//100+((height-10)*70//100)//3
        h_3=height*30//100+((height-10)*70//100)*2//3
        h_4=height*30//100+((height-10)*70//100)

        line_1=self.count_pix(h_1,perspective_transform)
        line_2=self.count_pix(h_2,perspective_transform)
        line_3=self.count_pix(h_3,perspective_transform)
        line_4=self.count_pix(h_4,perspective_transform)
     
        line_1_pt=list(line_1.keys())
        line_2_pt=list(line_2.keys())
        line_3_pt=list(line_3.keys())
        line_4_pt=list(line_4.keys())
        
        line_fit=[]
        line_fit.append(line_1_pt)
        line_fit.append(line_2_pt)
        line_fit.append(line_3_pt)
        line_fit.append(line_4_pt)
        #print('line_fit',line_fit)
        x=[]
        y=[]
        for i in range(len(line_fit)):
            if len(line_fit[i])!=0:
                x.append((line_fit[i][0][0]+line_fit[i][0][1])//2)
                y.append(line_fit[i][0][2])
        print(x,'  ',y)
        #print(y)

        #coefficients = np.polyfit(y, x, 2)
        #line_fit = np.poly1d(coefficients)

        #x_fit = np.int64(line_fit(y))
        #print(x_fit)
        #print(x_fit)
        #print(y)
        #print(perspective_transform)
        #print(coefficients)
        return x,y,perspective_transform

    def line_detection(self,file):
        x,y,perspective_transform=self.color_thresh(file)
        found_line=False
        coefficients=None
        if len(x)>=2:
            found_line=True
            coefficients = np.polyfit(y, x, 2)
            #line_fit = np.poly1d(coefficients)

            #x_fit = np.int64(line_fit(y))
        return found_line,coefficients,perspective_transform
    
    def cycle(self):
        #make modtion loop
        #images = glob.glob('./experiment/image_test*.jpg')
        i=0
        target_steering=0
        target_velocity=1
        while not rospy.is_shutdown():
        #for i in images:
            #frame = cv2.imread(i)
            #output_image_path = ImageFilter(i)
            #print('i',i)
            i=i+1
            
            #print('output_img',self.raw_img)
            if self.raw_img is not None:
                print('i',i)
                #print('output_img',self.raw_img)
                curr_found_line,curr_coeff,tranform_img=self.line_detection(self.raw_img)
                err=[]
                if curr_found_line is True:
                    self.coeff=curr_coeff
                    print("coeff",self.coeff)
                    err1,err2,xx,yy=self.error_compute(340,self.coeff,'dist')
                    err.append([i,err1])
                    #print('err  ',err1,'err  ',err2)
                    #cv2.circle(output_img, (x[0], y[0]), 10, (0, 0, 0), -1)
                    #cv2.circle(output_img, (x[1], y[1]), 10, (0, 0, 0), -1)
                    #cv2.circle(output_img, (x[2], y[2]), 10, (0, 0, 0), -1)
                    #cv2.circle(output_img, (x[3], y[3]), 10, (0, 0, 0), -1)
                    target_steering = self.pid.compute(err1)
                    self.prev_steering=target_steering
                else:            
                    target_steering=self.prev_steering
                '''

                if (len(err)%20==0):
                    #points = [[1, 2], [3, 4], [5, 6]]

                    # Extract x and y coordinates
                    x_coordinates, y_coordinates = zip(*err)

                    # Plot the points
                    plt.scatter(x_coordinates, y_coordinates, color='blue', marker='o', label='Points')

                    # Add labels and title
                    plt.xlabel('X-axis')
                    plt.ylabel('Y-axis')
                    plt.title('Scatter Plot of Points')

                    # Display legend
                    plt.legend()

                    # Display the plot
                    plt.show()
                '''
                target_velocity=1



                #cv2.circle(tranform_img, (xx, yy), 10, (0, 0, 0), -1)

                #cv2.imshow('hough', output_image_path)
                #v2.imshow('hsv_perspective', img)

                #cv2.imshow('hsv_perspective', tranform_img)
                #cv2.waitKey(0)
            
     #           print('steering_ang  ',target_steering)


                self.drive_msg.header.stamp = rospy.get_rostime()
                self.drive_msg.drive.steering_angle = target_steering
                self.ctrl_pub.publish(self.drive_msg)
                #newAckermannCmd = AckermannDrive()
                #newAckermannCmd.speed = target_velocity
                #newAckermannCmd.steering_angle = target_steering

                #self.controlPub.publish(newAckermannCmd)
                self.rate.sleep()
        
    def error_compute(self,y_ahead,coefficients,type_err='dist'):
        err=0
        height=480
        width=640
        print("Hello")
        print(coefficients)
        line_fit= np.poly1d(coefficients)
        x_pos=np.int64(line_fit(height-y_ahead))
        #if type_err=='dist':
        err1=x_pos-width//2
        #else:
        err2=np.degrees(np.arctan((x_pos-width//2)/y_ahead))
        return err1,err2,x_pos,height-y_ahead

def plot_pid():
    

    # Coordinates of the points
    points = [[1, 2], [3, 4], [5, 6]]

    # Extract x and y coordinates
    x_coordinates, y_coordinates = zip(*points)

    # Plot the points
    plt.scatter(x_coordinates, y_coordinates, color='blue', marker='o', label='Points')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Scatter Plot of Points')

    # Display legend
    plt.legend()

    # Display the plot
    plt.show()

def main():
    rospy.init_node('controller', anonymous=True)
    model=Motion()
    #model.cycle()
    #rospy.Subscriber(lidar_topic, Scan, lidar_callback)
    # Keep the program alive
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
    #model=Motion()
    #model.cycle()
    #plot_pid()

