#!/usr/bin/env python3

#Python Libs
import sys, time

#numpy
import numpy as np

#OpenCV
import cv2
from cv_bridge import CvBridge

#ROS Libraries
import rospy
import roslib

#ROS Message Types
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()

        #### REMEMBER TO CHANGE THE TOPIC NAME! #####
        self.image_sub = rospy.Subscriber('/madtp/camera_node/image/compressed', CompressedImage, self.image_callback, queue_size=1)
        #############################################

        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert to opencv image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        #### YOUR CODE GOES HERE ####
        # Crop the image to show only the road
        img_cropped = img[200:480, 0:640]  # Adjust these values as per your requirement

        # Define white color range in RGB
        lower_white = np.array([200, 200, 200])
        upper_white = np.array([255, 255, 255])

        # Define yellow color range in RGB
        lower_yellow = np.array([150, 150, 0])  # Adjusted yellow range
        upper_yellow = np.array([255, 255, 150])  # Adjusted yellow range

        # Filter for white pixels
        white_mask = cv2.inRange(img_cropped, lower_white, upper_white)

        # Filter for yellow pixels
        yellow_mask = cv2.inRange(img_cropped, lower_yellow, upper_yellow)

        # Apply Canny Edge Detector
        edges = cv2.Canny(img_cropped, 100, 200)

        # Apply Hough Transform to the White-filtered image
        lines_white = cv2.HoughLinesP(white_mask, 1, np.pi / 180, 50, maxLineGap=10)

        # Apply Hough Transform to the Yellow-filtered image
        lines_yellow = cv2.HoughLinesP(yellow_mask, 1, np.pi / 180, 50, maxLineGap=10)

        # Draw lines found on both Hough Transforms on the cropped image
        img_out = img_cropped.copy()
        if lines_white is not None:
            for line in lines_white:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_out, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                cv2.line(img_out, (x1, y1), (x2, y2), (0, 255, 255), 2)

        #############################

        # Show images in separate windows
        cv2.imshow('White-filtered', white_mask)
        cv2.imshow('Yellow-filtered', yellow_mask)
        cv2.imshow('img_out', img_out)
        cv2.waitKey(1)

    def run(self):
        rospy.spin() # Spin forever but listen to message callbacks

if __name__ == "__main__":
    try:
        lane_detector_instance = Lane_Detector()
        lane_detector_instance.run()
    except rospy.ROSInterruptException:
        pass
