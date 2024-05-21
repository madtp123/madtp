#!/usr/bin/env python3

import rospy
import math

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

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/madtp/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/madtp/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        self.object_detected = False
        self.seek_object_mode = True
        self.prev_object_detected = False  # To track the previous object detection state

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
        if len(detections) == 0:
            self.object_detected = False
            self.seek_object_mode = True
            print("No objects detected, seeking mode enabled.")
        else:
            self.object_detected = True
            self.seek_object_mode = False
            x = detections[0].transform.translation.x
            y = detections[0].transform.translation.y
            z = detections[0].transform.translation.z
            rospy.loginfo("Object detected at x,y,z: %f %f %f", x, y, z)

        # Publish velocity commands
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if self.seek_object_mode:
            # "Seek an object" Feature
            cmd_msg.v = 0.0  # Stop linear motion
            cmd_msg.omega = 8.0  # Rotate at a fixed rate to search for objects
            print("Seeking object, rotating in place.")
        else:
            # "Look at the Object" Feature
            if self.object_detected and not self.prev_object_detected:
                # Object detected after not being detected
                cmd_msg.v = 0.0  # Stop linear and angular motion
                cmd_msg.omega = 0.0
                print("Object detected, stopping motion.")
            elif not self.object_detected and self.prev_object_detected:
                # Object removed after being detected
                cmd_msg.v = 0.0  # Stop linear motion
                cmd_msg.omega = 8.0  # Rotate at a fixed rate to search for objects
                print("Object removed, seeking mode enabled.")
            else:
                # Object is still in view
                cmd_msg.v = 0.0  # Stop linear motion
                cmd_msg.omega = -y / (x + 0.00001)  # Rotate to keep the object in the center of the field of view
                print("Looking at the object, rotating to keep it in view.")

        self.prev_object_detected = self.object_detected  # Update the previous object detection state
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("Published velocity command: v=%.2f, omega=%.2f", cmd_msg.v, cmd_msg.omega)
        #############################

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
