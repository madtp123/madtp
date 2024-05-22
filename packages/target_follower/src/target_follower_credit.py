#!/usr/bin/env python3

import rospy
import math
import time

from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.loginfo("Target follower node started.")

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "madtp" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/madtp/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/madtp/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        self.object_detected = False
        self.seek_object_mode = False  # Start in lane following mode
        self.prev_object_detected = False  # To track the previous object detection state
        self.stop_sign_detected = False
        self.stop_sign_wait_start_time = None

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        rospy.loginfo("Received message from AprilTag detector node.")
        self.move_robot(msg.detections)

    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        #### YOUR CODE GOES HERE ####
        stop_sign_detected = False
        for detection in detections:
            if detection.tag_id == 18:  # Stop sign tag ID
                stop_sign_detected = True
                break

        if stop_sign_detected:
            self.stop_sign_detected = True
            self.stop_sign_wait_start_time = rospy.get_time()
            print("Stop sign detected, stopping for 5 seconds.")
        elif self.stop_sign_detected:
            current_time = rospy.get_time()
            if current_time - self.stop_sign_wait_start_time >= 5.0:  # Wait for 5 seconds
                self.stop_sign_detected = False
                print("Resuming lane following.")

        # Publish velocity commands
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if self.stop_sign_detected:
            cmd_msg.v = 0.0  # Stop linear motion
            cmd_msg.omega = 0.0  # Stop angular motion
        else:
            # Lane following behavior
            cmd_msg.v = 0.3  # Set a constant linear velocity
            cmd_msg.omega = 0.0  # Set angular velocity to zero for lane following

        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("Published velocity command: v=%.2f, omega=%.2f", cmd_msg.v, cmd_msg.omega)
        #############################

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass

