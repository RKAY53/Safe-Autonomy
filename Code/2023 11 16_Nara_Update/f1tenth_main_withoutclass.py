#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image,LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math

import numpy as np
from numpy import linalg as LA

import glob
from skimage import morphology
import time
import matplotlib.pyplot as plt

def image_callback(ros_image):
    bridge = CvBridge()
  
    try:
        # Convert the ROS Image message to a format OpenCV understands
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        #z16
        
    except CvBridgeError as e:
        print(e)

    # Use the current time as the file name
    file_name = "image_ht{}.jpg".format(time.strftime("%Y%m%d-%H%M%S"))
    # Save the image using OpenCV
    # print(cv_image.shape)
    cv_image = ImageFilter(cv_image)

    #print(cv_image.shape)
    cv2.imwrite(file_name, cv_image)
    print("Haha")
    # cv2.imshow('show', cv_image)
    # # print(cv_image)
    # cv2.waitKey(0)

def d_image_callback(ros_image):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a format OpenCV understands
        cv_image = bridge.imgmsg_to_cv2(ros_image,desired_encoding='passthrough')
        #z16
    except CvBridgeError as e:
        print(e)

    # Use the current time as the file name
    file_name = "image_cal{}.jpg".format(time.strftime("%Y%m%d-%H%M%S"))
    # Save the image using OpenCV
    # print(cv_image.shape)
    # cv_image = ImageFilter(cv_image)
    print(cv_image)
    cv2.imshow('show2', cv_image)
    cv2.waitKey(0)
    #print('shape   ',cv_image.shape)

def main():
    rospy.init_node('image_saver', anonymous=True)
    lidar_topic = "/scan"
    image_topic = "/D435I/color/image_raw"  # Adjust to match the topic for your camera
    #d_image_topic = "/D435I/depth/image_rect_raw"  # Adjust to match the topic for your camera

    rospy.Subscriber(image_topic, Image, image_callback)
    #r#ospy.Subscriber(d_image_topic, Image, d_image_callback)
    model=Motion()
    model.cycle()
    #rospy.Subscriber(lidar_topic, Scan, lidar_callback)
    # Keep the program alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

def ImageFilter(image_path):
    # Read the image
    frame = cv2.imread(image_path)
    height, width = frame.shape[:2]

    #print(frame.shape)
    # Coordinates for perspective transformation
    '''
    source = np.float32([
        [60, 420],
        [1220,420],
        [0, 640],
        [1280, 640]
            ])
    dest = np.float32([
        [0, 0],
        [1280, 0],
        [0, 720],
        [1280, 720]
    ])
    '''
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


    # Convert to HSV color space
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range for yellow color and create mask
    # Adjust the lower and upper bounds to properly capture the yellow lane
    low_yellow = np.array([18, 94, 140])  # These values are approximations for yellow
    high_yellow = np.array([48, 255, 255])
    mask_yellow = cv2.inRange(img_hsv, low_yellow, high_yellow)

    # Apply Gaussian blur to the mask
    blur_yellow = cv2.GaussianBlur(mask_yellow, (11, 11), 0)

    # Use Canny edge detection
    edges = cv2.Canny(blur_yellow, 50, 150)

    perspective_transform = cv2.warpPerspective(edges, M, (width, height))

    # Use Hough Transform to detect lines in the edge-detected image
    lines = cv2.HoughLinesP(perspective_transform, 1, np.pi / 180, threshold=150, minLineLength=50, maxLineGap=250)
    #print(lines)
    #print(lines.shape)


        # Function to extrapolate the lines
    def extrapolate_lines(image, lines):
        #print(lines)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    # slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 0
                    # intercept = y1 - (slope * x1)
                    
                    # if abs(slope) > 0.5:  # This filters out nearly horizontal lines
                    #     # Set new start and end points for extrapolation
                    #     y1_new = image.shape[0]  # Bottom of the image
                    #     x1_new = int((y1_new - intercept) / slope) if slope != 0 else x1
                    #     y2_new = int(image.shape[0] * 0.6)  # Approximately the horizon line
                    #     x2_new = int((y2_new - intercept) / slope) if slope != 0 else x2
                        
                    #     # Calculate midpoint
                    #     mx = (x1_new + x2_new) // 2
                    #     my = (y1_new + y2_new) // 2
                        
                    #     # Determine shift for 45-degree angle
                    #     shift_length = 130  # This value can be adjusted
                        
                    #     # Calculate new midpoint for 45-degree left bend
                    #     mx_new = mx - shift_length // np.sqrt(2)
                    #     my_new = my - shift_length // np.sqrt(2)
                        
                    #     # Draw the two new line segments
                    #     cv2.line(image, (x1_new, y1_new), (int(mx_new), int(my_new)), (255, 255, 255), 3)
                    #     cv2.line(image, (int(mx_new), int(my_new)), (x2_new, y2_new), (255, 255, 255), 3)
                    # Draw the two new line segments
                    cv2.line(image, (x1, y1), (int(x2), int(y2)), (255, 255, 255), 3)
                        
        return image


    # Draw the extrapolated lines on the image
    # lane_image = np.copy(perspective_transform)
    lane_image = extrapolate_lines(perspective_transform, lines)/255
    #print(lane_image)
    #print(lane_image.shape)
    # Save the result
    #output_path = 'output.jpg'
    #cv2.imwrite(output_path, lane_image)
    #print(np.max(lane_image))
    binaryImage = morphology.remove_small_objects(lane_image.astype('bool'),min_size=100,connectivity=2)
    binaryImage = binaryImage.astype('double')

    #print(binaryImage)
    last_row = binaryImage[-1, :]

    # Find indices where the last row has value 1
    indices = np.where(last_row == 1)[0]
    #print(indices)
    # Count the number of such indices
    #count = len(indices)
    return binaryImage

def count_pix(h,perspective_transform):
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

def color_thresh(file):
        img = cv2.imread(file)
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

        line_1=count_pix(h_1,perspective_transform)
        line_2=count_pix(h_2,perspective_transform)
        line_3=count_pix(h_3,perspective_transform)
        line_4=count_pix(h_4,perspective_transform)
     
        line_1_pt=list(line_1.keys())
        line_2_pt=list(line_2.keys())
        line_3_pt=list(line_3.keys())
        line_4_pt=list(line_4.keys())

        pt_1=[(line_1_pt[0][0]+line_1_pt[0][1])//2,line_1_pt[0][2]]
        pt_2=[(line_2_pt[0][0]+line_2_pt[0][1])//2,line_2_pt[0][2]]
        pt_3=[(line_3_pt[0][0]+line_3_pt[0][1])//2,line_3_pt[0][2]]
        pt_4=[(line_4_pt[0][0]+line_4_pt[0][1])//2,line_4_pt[0][2]]

        #print('1:  ',line_1, pt_1)
        #print('2:  ',line_2, pt_2)
        #print('3:  ',line_3, pt_3)
        #print('4:  ',line_4, pt_4)
        x = [pt_1[0],pt_2[0],pt_3[0],pt_4[0]]
        y = [pt_1[1],pt_2[1],pt_3[1],pt_4[1]]
        coefficients = np.polyfit(y, x, 2)
        line_fit = np.poly1d(coefficients)

        x_fit = np.int64(line_fit(y))
        #print(x_fit)

        return x_fit,y,perspective_transform,line_fit

def error_compute(y_ahead,line_fit,type_err='dist'):
    err=0
    height=480
    width=640
    x_pos=np.int64(line_fit(height-y_ahead))
    #if type_err=='dist':
    err1=x_pos-width//2
    #else:
    err2=np.degrees(np.arctan((x_pos-width//2)/y_ahead))
    return err1,err2,x_pos,height-y_ahead

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
        self.pid = PID(KP,KI,KD)
        self.kpe = np.array([])
        self.kde = np.array([])
        self.kie = np.array([])
        #self.thrst = np.array([])
    def cycle(self):
        #make modtion loop
        images = glob.glob('./experiment/image_test*.jpg')
        #while(True):
        for i in images:
            #frame = cv2.imread(i)
            output_image_path = ImageFilter(i)
            x,y,output_image_path3,line_fit = color_thresh(i)
            err1,err2,xx,yy=error_compute(340,line_fit,'dist')
            #print('err  ',err1,'err  ',err2)
            #cv2.circle(output_image_path3, (x[0], y[0]), 10, (0, 0, 0), -1)
            #cv2.circle(output_image_path3, (x[1], y[1]), 10, (0, 0, 0), -1)
            #cv2.circle(output_image_path3, (x[2], y[2]), 10, (0, 0, 0), -1)
            #cv2.circle(output_image_path3, (x[3], y[3]), 10, (0, 0, 0), -1)
            cv2.circle(output_image_path3, (xx, yy), 10, (0, 0, 0), -1)

            cv2.imshow('hough', output_image_path)
            cv2.imshow('hsv_perspective', output_image_path3)
            #cv2.waitKey(0)
            steering_ang = self.pid.compute(err1)

            print('steering_ang  ',steering_ang)
            #publish steering angle to car, may also create logic to control speed
        ctr=self.pid.get_ctr()
        print('ctr  ',ctr)
        x_values, y_values = zip(*ctr)

        # Plot the points
        #plt.scatter(x_coordinates, y_coordinates, color='blue', marker='o', label='Points')
        plt.plot(x_values, y_values, marker='o', linestyle='-', color='blue', label='Line with Markers')

        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Scatter Plot of Points')

        # Display legend
        plt.legend()

        # Display the plot
        plt.show()
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

if __name__ == '__main__':
    main()
    #model=Motion()
    #model.cycle()
    #plot_pid()


    '''
    images = glob.glob('./experiment/image_test*.jpg')


    
    for i in images:
        frame = cv2.imread(i)
        output_image_path = ImageFilter(i)
        x,y,output_image_path3,line_fit = color_thresh(i)
        err1,err2,xx,yy=error_compute(340,line_fit,'dist')
        print('err  ',err1,'err  ',err2)
        #cv2.circle(output_image_path3, (x[0], y[0]), 10, (0, 0, 0), -1)
        #cv2.circle(output_image_path3, (x[1], y[1]), 10, (0, 0, 0), -1)
        #cv2.circle(output_image_path3, (x[2], y[2]), 10, (0, 0, 0), -1)
        #cv2.circle(output_image_path3, (x[3], y[3]), 10, (0, 0, 0), -1)
        cv2.circle(output_image_path3, (xx, yy), 10, (0, 0, 0), -1)

        cv2.imshow('hough', output_image_path)
        #cv2.imshow('hsv', output_image_path2)
        cv2.imshow('hsv_perspective', output_image_path3)
        cv2.waitKey(0)

    
        #img
    output_image_path = ImageFilter(images[0])
    x,y,output_image_path3,line_fit = color_thresh(images[0])
    cv2.circle(output_image_path3, (x[0], y[0]), 10, (0, 0, 0), -1)
    cv2.circle(output_image_path3, (x[1], y[1]), 10, (0, 0, 0), -1)
    cv2.circle(output_image_path3, (x[2], y[2]), 10, (0, 0, 0), -1)
    cv2.circle(output_image_path3, (x[3], y[3]), 10, (0, 0, 0), -1)


    cv2.imshow('out', output_image_path)
    #cv2.imshow('raw', output_image_path2)
    cv2.imshow('hsv_perspective', output_image_path3)

    cv2.waitKey(0)
        # output_image_path
    '''
