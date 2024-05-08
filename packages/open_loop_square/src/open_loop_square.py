#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        # Initialize Pub/Subs
        self.pub = rospy.Publisher('/madtp/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/madtp/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # Robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
        rospy.spin() # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):
        # Move forward for the side of the square
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(2)  # Adjust this duration to make sure the robot moves straight enough
        
        self.stop_robot()  # Stop after moving straight
        
        rospy.sleep(1)  # Wait a moment before starting the turn
        
        # Turn left
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0  # Stop the forward motion
        self.cmd_msg.omega = 0.5  # Angular velocity for turning
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turn!")
        rospy.sleep(1)  # Adjust this duration to make a 90-degree turn
        
        self.stop_robot()  # Stop after turning
        
        ######################
        
        # Repeat the same process for the remaining sides of the square
        # Move forward for the second side
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(2)  # Adjust this duration to make sure the robot moves straight enough
        
        self.stop_robot()  # Stop after moving straight
        
        rospy.sleep(1)  # Wait a moment before starting the turn
        
        # Turn left again
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0  # Stop the forward motion
        self.cmd_msg.omega = 0.5  # Angular velocity for turning
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turn!")
        rospy.sleep(1)  # Adjust this duration to make a 90-degree turn
        
        self.stop_robot()  # Stop after turning
        
        # Repeat the same process for the remaining sides of the square

        # Move forward for the third side
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(2)  # Adjust this duration to make sure the robot moves straight enough
        
        self.stop_robot()  # Stop after moving straight
        
        rospy.sleep(1)  # Wait a moment before starting the turn
        
        # Turn left again
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0  # Stop the forward motion
        self.cmd_msg.omega = 0.5  # Angular velocity for turning
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turn!")
        rospy.sleep(1)  # Adjust this duration to make a 90-degree turn
        
        self.stop_robot()  # Stop after turning

        # Repeat the same process for the fourth side
        # Move forward for the fourth side
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.5  # straight line velocity
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Forward!")
        rospy.sleep(2)  # Adjust this duration to make sure the robot moves straight enough
        
        self.stop_robot()  # Stop after moving straight
        
        rospy.sleep(1)  # Wait a moment before starting the turn
        
        # Turn left again
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0  # Stop the forward motion
        self.cmd_msg.omega = 0.5  # Angular velocity for turning
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Turn!")
        rospy.sleep(1)  # Adjust this duration to make a 90-degree turn
        
        self.stop_robot()  # Stop after turning
        
        ######################
        
        self.stop_robot()  # Ensure the robot stops at the end

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass

