#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image,LaserScan
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import math
from numpy import linalg as LA



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

    #rospy.Subscriber(lidar_topic, Scan, lidar_callback)
    # Keep the program alive
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
'''
def ImageFilter(frame):
    #video
    source = np.float32([[400, 180], [800, 180], [400, 540], [800, 540]])
    dest = np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]])
    M = cv2.getPerspectiveTransform(source, dest)

    while(True):
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
        break
    return edges
'''
# The provided code is intended to be run in a local environment because it includes displaying the image in a window
# which cannot be executed in this notebook environment. However, I will modify the code to target the yellow lane,
# and instead of displaying the image in a window, I will output the image file so you can see the result.
import cv2
import numpy as np 
import glob

def ImageFilter(frame):
    # Read the image
    # frame = cv2.imread(image_path)
    height, width = frame.shape[:2]

    # Coordinates for perspective transformation
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


        # Function to extrapolate the lines
    def extrapolate_lines(image, lines):
        print(lines)
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
    lane_image = extrapolate_lines(perspective_transform, lines)

    # Save the result
    # output_path = 'output.jpg'
    # cv2.imwrite(output_path, lane_image)
    return lane_image

# Call the function with the image file name
# images = glob.glob('./image_test*.jpg')
# images = glob.glob('lane1.jpg')

# for i in images:
#     frame = cv2.imread(i)
#     output_image_path = ImageFilter(i)
#     cv2.imshow('out', output_image_path)
#     cv2.imshow('raw', frame)
#     cv2.waitKey(0)
# output_image_path



            

#sensor_msgs/LaserScan


if __name__ == '__main__':
    main()