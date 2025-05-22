import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 5.0
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

END_OF_GRID = 5.0  # Define the x-coordinate where the grid ends
SAFE_DISTANCE = 0.5  # Define the minimum safe distance between the robots

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    return input

def checkLinearLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
        vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel, turtlebot3_model):
    if turtlebot3_model == "burger":
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
        vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    return vel

class Actuate(Node):
    def __init__(self):
        super().__init__('actuate_node')

        # Publishers for all robots' /cmd_vel topics
        self.turtlebot1_pub = self.create_publisher(Twist, '/turtlebot1_ns/cmd_vel', 10)
        self.turtlebot2_pub = self.create_publisher(Twist, '/turtlebot2_ns/cmd_vel', 10)
        self.main_robot_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Main robot's topic

        # Subscribers for /odom topics to track positions
        self.main_odom_sub = self.create_subscription(Odometry, '/odom', self.main_robot_odom_callback, 10)
        self.turtlebot1_odom_sub = self.create_subscription(Odometry, '/turtlebot1_ns/odom', self.turtlebot1_odom_callback, 10)
        self.turtlebot2_odom_sub = self.create_subscription(Odometry, '/turtlebot2_ns/odom', self.turtlebot2_odom_callback, 10)

        # Subscriber for control messages from the dashboard
        self.sub_control = self.create_subscription(String, '/control', self.key_callback, 10)

        # Retrieve the turtlebot model parameter
        self.declare_parameter('model', 'burger')
        self.turtlebot3_model = self.get_parameter('model').get_parameter_value().string_value

        # Track the active robot (0 = main, 1 = turtlebot1, 2 = turtlebot2)
        self.active_robot = 2

        # Robot positions (including x and y)
        self.main_robot_position = [0.0, 0.0]  # [x, y]
        self.turtlebot1_position = [0.0, 0.0]  # [x, y]
        self.turtlebot2_position = [0.0, 0.0]  # [x, y]

        # Block movement flags
        self.blocked_x = False  # Forward/backward blocked
        self.blocked_y = False  # Left/right blocked

        # Send initial zero velocity to all robots
        self.send_initial_zero_velocity()

    def publish_twist_to_active(self, twist):
        """Publish twist to active robot after applying constraints."""
        if self.active_robot == 1:
            self.apply_directional_constraints(twist, self.turtlebot1_position, self.turtlebot2_position)
        elif self.active_robot == 0:
            self.apply_directional_constraints(twist, self.main_robot_position, self.turtlebot1_position)

        # Publish the twist to the currently active robot
        if self.active_robot == 0:
            self.main_robot_pub.publish(twist)
        elif self.active_robot == 1:
            self.turtlebot1_pub.publish(twist)
        elif self.active_robot == 2:
            self.turtlebot2_pub.publish(twist)

    def check_safe_distance(self, pos1, pos2):
        """Check if two robots are within SAFE_DISTANCE."""
        distance_x = abs(pos1[0] - pos2[0])  # Distance in x direction
        distance_y = abs(pos1[1] - pos2[1])  # Distance in y direction
        return distance_x < SAFE_DISTANCE, distance_y < SAFE_DISTANCE

    def apply_directional_constraints(self, twist, active_pos, other_pos):
        """Block movement only in the direction of potential collision."""
        blocked_x, blocked_y = self.check_safe_distance(active_pos, other_pos)

        if blocked_x and twist.linear.x != 0.0:  # Only block x if moving forward/backward
            twist.linear.x = 0.0
        if blocked_y and twist.angular.z != 0.0:  # Only block y if turning left/right
            twist.angular.z = 0.0

    def send_initial_zero_velocity(self):
        # Create a zero Twist message
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.linear.y = 0.0
        zero_twist.linear.z = 0.0
        zero_twist.angular.x = 0.0
        zero_twist.angular.y = 0.0
        zero_twist.angular.z = 0.0

        # Publish zero Twist to all robots
        self.main_robot_pub.publish(zero_twist)
        self.turtlebot1_pub.publish(zero_twist)
        self.turtlebot2_pub.publish(zero_twist)

    def key_callback(self, msg):
        twist = Twist()

        # Allow movement if not blocked in the corresponding direction
        if msg.data == 'w' and not self.blocked_x:  # Move forward
            twist.linear.x = 0.5
        elif msg.data == 's' and not self.blocked_x:  # Move backward
            twist.linear.x = -0.5
        elif msg.data == 'a' and not self.blocked_y:  # Turn left
            twist.angular.z = 1.0
        elif msg.data == 'd' and not self.blocked_y:  # Turn right
            twist.angular.z = -1.0
        elif msg.data == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Send the command only to the active robot
        self.publish_twist_to_active(twist)

    def stop_robot(self, pub):
        """Publish zero velocity to stop a robot."""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        pub.publish(stop_twist)

    def main_robot_odom_callback(self, msg):
        self.main_robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # Check if the main robot reached the end of the grid
        if self.main_robot_position[0] >= END_OF_GRID:
            print("Main robot reached the end")
            self.stop_robot(self.main_robot_pub)  # Stop the main robot
            self.active_robot = -1  # No more robots to control

    def turtlebot1_odom_callback(self, msg):
        self.turtlebot1_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # Check if turtlebot1 reached the end of the grid
        if self.turtlebot1_position[0] >= END_OF_GRID:
            print("Turtlebot1 reached the end")
            self.stop_robot(self.turtlebot1_pub)  # Stop turtlebot1
            self.active_robot = 0  # Switch control to turtlebot2

    def turtlebot2_odom_callback(self, msg):
        self.turtlebot2_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        # Check if turtlebot2 reached
        # Check if turtlebot2 reached the end of the grid (no further robots)
        if self.turtlebot2_position[0] >= END_OF_GRID:
            print("Turtlebot2 reached the end")
            self.stop_robot(self.turtlebot2_pub)  # Stop turtlebot2
            self.active_robot = 1  # Switch control to turtlebot1

rclpy.init()
node = Actuate()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
