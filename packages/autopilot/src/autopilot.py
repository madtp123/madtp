#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
 
class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)
        self.robot_state = "LANE_FOLLOWING"
        self.ignore_tags = False
        self.ignore_timer = None
        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
 
        self.cmd_vel_pub = rospy.Publisher('/madtp/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/madtp/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/madtp/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
 
        rospy.spin()  # Spin forever but listen to message callbacks
 
    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_tags:
            return
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
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)
 
    def move_robot(self, detections):
        if len(detections) == 0:
            return
        # Get the first detected AprilTag
        detection = detections[0]
        # Extract the tag ID and pose information
        tag_id = detection.tag_id
        tag_translation = detection.transform.translation
        tag_rotation = detection.transform.rotation
        # Access the position and orientation
        tag_position_x = tag_translation.x
        tag_position_y = tag_translation.y
        tag_position_z = tag_translation.z
        tag_orientation_x = tag_rotation.x
        tag_orientation_y = tag_rotation.y
        tag_orientation_z = tag_rotation.z
        tag_orientation_w = tag_rotation.w
        # Define the stop sign tag ID (replace with your actual stop sign tag ID)
        stop_sign_id = 10
        # Check if the detected tag is a stop sign
        if tag_id == stop_sign_id:
            rospy.loginfo("Stop sign detected. Stopping the robot.")
            self.set_state("NORMAL_JOYSTICK_CONTROL")  # Stop Lane Following
            self.stop_robot()  # Stop the robot
            rospy.sleep(5)  # Wait for 5 seconds
            self.ignore_tags = True  # Ignore AprilTag detections for a while
            self.ignore_timer = rospy.Timer(rospy.Duration(3), self.ignore_timer_callback, oneshot=True)
            rospy.loginfo("Resuming lane following.")
            self.set_state("LANE_FOLLOWING")  # Go back to lane following
        else:
            rospy.loginfo("Non-stop sign tag detected. Ignoring.")
 
    def ignore_timer_callback(self, event):
        self.ignore_tags = False  # Stop ignoring AprilTag detections
 
if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
