import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

END_OF_GRID = 5.0  # Define the x-coordinate where the grid ends
SAFE_DISTANCE = 0.5  # Define the minimum safe distance between the robots

class Actuate(Node):
    def __init__(self):
        super().__init__('actuate_node')

        # Publishers for Dog Robot and TurtleBot
        self.dog_robot_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turtlebot_pub = self.create_publisher(Twist, '/tb3/cmd_vel', 10)

        # Subscribers for odometry topics to track positions
        self.dog_robot_odom_sub = self.create_subscription(Odometry, '/odom', self.dog_robot_odom_callback, 10)
        self.turtlebot_odom_sub = self.create_subscription(Odometry, '/tb3/odom', self.turtlebot_odom_callback, 10)

        # Subscriber for control messages from the dashboard
        self.sub_control = self.create_subscription(String, '/control', self.key_callback, 10)

        # Track the active robot (0 = Dog Robot, 1 = TurtleBot)
        self.active_robot = 1

        # Robot positions (including x and y)
        self.dog_robot_position = [0.0, 0.0]  # [x, y]
        self.turtlebot_position = [0.0, 0.0]  # [x, y]

        # Send initial zero velocity to both robots
        self.send_initial_zero_velocity()

    def publish_twist_to_active(self, twist):
        """Publish twist to the active robot."""
        if self.active_robot == 0:
            self.dog_robot_pub.publish(twist)
        elif self.active_robot == 1:
            self.turtlebot_pub.publish(twist)

    def send_initial_zero_velocity(self):
        """Send zero velocity to all robots."""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0

        self.dog_robot_pub.publish(zero_twist)
        self.turtlebot_pub.publish(zero_twist)

    def key_callback(self, msg):
        """Handle keyboard input and publish velocity commands."""
        twist = Twist()

        if msg.data == 'w':  # Move forward
            twist.linear.x = 0.5
        elif msg.data == 's':  # Move backward
            twist.linear.x = -0.5
        elif msg.data == 'a':  # Turn left
            twist.angular.z = 1.0
        elif msg.data == 'd':  # Turn right
            twist.angular.z = -1.0
        elif msg.data == ' ':  # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish the command to the active robot
        self.publish_twist_to_active(twist)

    def stop_robot(self, pub):
        """Publish zero velocity to stop a robot."""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        pub.publish(stop_twist)

    def dog_robot_odom_callback(self, msg):
        """Callback for the Dog Robot's odometry."""
        self.dog_robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        if self.dog_robot_position[0] >= END_OF_GRID:
            self.get_logger().info("Dog Robot reached the end")
            self.stop_robot(self.dog_robot_pub)  # Stop the Dog Robot
            self.active_robot = -1  # Switch control to TurtleBot

    def turtlebot_odom_callback(self, msg):
        """Callback for the TurtleBot's odometry."""
        self.turtlebot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

        if self.turtlebot_position[0] >= END_OF_GRID:
            self.get_logger().info("TurtleBot reached the end")
            self.stop_robot(self.turtlebot_pub)  # Stop the TurtleBot
            self.active_robot = 0  # No more robots to control


def main(args=None):
    rclpy.init(args=args)
    node = Actuate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
