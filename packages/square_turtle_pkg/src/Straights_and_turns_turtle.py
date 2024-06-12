import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavigator:
    def __init__(self):
        # Initialize node and class variables
        rospy.init_node('turtle_navigator_node', anonymous=True)
        self.pose = None
        self.goal = None
        self.distance_traveled = 0.0

        # Set up subscriber
        rospy.Subscriber("/turtle1/pose", Pose, self.update_state)
        rospy.Subscriber("/goal", Pose, self.update_goal)

        # Set up publisher
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Set up timer for main control loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Turtle Navigator Node Initialized")
        rospy.spin()

    def update_state(self, msg):
        # Update current pose and calculate distance traveled
        if self.pose is not None:
            dx = msg.x - self.pose.x
            dy = msg.y - self.pose.y
            self.distance_traveled += math.sqrt(dx**2 + dy**2)
        self.pose = msg

    def update_goal(self, msg):
        # Update goal position
        self.goal = msg

    def control_loop(self, event):
        # Main control loop for navigation
        twist_msg = Twist()

        if self.goal is not None:
            # Calculate desired angle to face the goal
            goal_angle = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

            # Rotate to face the goal
            twist_msg.angular.z = self.rotate_to_angle(goal_angle)

            # Move towards the goal
            goal_distance = math.sqrt((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)
            twist_msg.linear.x = self.move_to_goal(goal_distance)

        self.cmd_vel_pub.publish(twist_msg)

    def rotate_to_angle(self, goal_angle):
        # Rotate the turtle to face the desired angle
        angular_vel = 1.0 if goal_angle > 0 else -1.0
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < abs(goal_angle) / 180 * math.pi:
            rospy.sleep(0.01)
        return 0.0

    def move_to_goal(self, goal_distance):
        # Move the turtle towards the goal
        if self.distance_traveled < goal_distance:
            return 1.0
        else:
            self.goal = None
            self.distance_traveled = 0.0
            return 0.0

if __name__ == '__main__':
    try:
        TurtleNavigator()
    except rospy.ROSInterruptException:
        pass