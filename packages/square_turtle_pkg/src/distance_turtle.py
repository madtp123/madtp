#!/usr/bin/env python3

# Import Dependencies
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math 

class DistanceReader:
    def __init__(self):
        # Initialize the node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Initialize subscriber: input the topic name, message type and callback signature  
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        # Initialize publisher: input the topic name, message type and msg queue size
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Previous position variables
        self.prev_x = None
        self.prev_y = None

        # Total distance traveled
        self.total_distance = 0.0

        # Set the publishing rate to 0.5 Hz (once every 2 seconds)
        self.publish_rate = rospy.Rate(0.5)

        # This blocking function call keeps Python from exiting until the node is stopped
        rospy.spin()

    # Whenever a message is received from the specified subscriber, this function will be called
    def callback(self, msg):
        # If previous position exists, calculate distance and publish
        if self.prev_x is not None and self.prev_y is not None:
            # Calculate Euclidean distance
            distance = math.sqrt((msg.x - self.prev_x)**2 + (msg.y - self.prev_y)**2)

            # Add the distance to total distance traveled
            self.total_distance += distance

            # Print the total distance traveled
            print("Total Distance Traveled:", self.total_distance)

            # Publish total distance traveled at the specified rate
            if self.publish_rate.remaining() == rospy.Duration(0):
                self.distance_publisher.publish(self.total_distance)
                self.publish_rate.sleep()

        # Update previous position
        self.prev_x = msg.x
        self.prev_y = msg.y

if __name__ == '__main__': 
    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
