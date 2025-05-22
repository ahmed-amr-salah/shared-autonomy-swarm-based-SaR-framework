import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Actuate(Node):
    def __init__(self):
        super().__init__('actuate_node')

        # Publishers for both robots
        self.turtlebot_pub = self.create_publisher(Twist, '/turtlebot_ns/cmd_vel', 10)
        self.dog_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher for the dog robot

        # Subscriber for control messages from the dashboard
        self.sub_control = self.create_subscription(String, '/control', self.key_callback, 10)

        # Send initial zero velocity to both robots
        self.send_initial_zero_velocity()

    def send_initial_zero_velocity(self):
        """Send zero velocity to both robots."""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0

        self.turtlebot_pub.publish(zero_twist)
        self.dog_pub.publish(zero_twist)  # Send zero velocity to the dog robot

    def key_callback(self, msg):
        """Handle keyboard input and publish velocity commands to both robots."""
        turtlebot_twist = Twist()
        dog_twist = Twist()

        if msg.data == 'w':  # Move forward
            turtlebot_twist.linear.x = 0.3  # TurtleBot moves slower
            dog_twist.linear.x = 0.5  # Dog robot moves faster
        elif msg.data == 's':  # Move backward
            turtlebot_twist.linear.x = -0.3  # TurtleBot moves slower
            dog_twist.linear.x = -0.5  # Dog robot moves faster
        elif msg.data == 'a':  # Turn left
            turtlebot_twist.angular.z = 1.0  # Same angular velocity for both
            dog_twist.angular.z = 1.0
        elif msg.data == 'd':  # Turn right
            turtlebot_twist.angular.z = -1.0  # Same angular velocity for both
            dog_twist.angular.z = -1.0
        elif msg.data == ' ':  # Stop
            turtlebot_twist.linear.x = 0.0
            turtlebot_twist.angular.z = 0.0
            dog_twist.linear.x = 0.0
            dog_twist.angular.z = 0.0

        # Publish the command to both robots
        self.turtlebot_pub.publish(turtlebot_twist)
        self.dog_pub.publish(dog_twist)

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
