import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class Actuate(Node):
    def __init__(self):
        super().__init__('actuate_node')

        # Publisher for the TurtleBot
        #self.turtlebot_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turtlebot_pub = self.create_publisher(Twist, '/robot_0/HumanControl', 10)
        self.boolean_pub = self.create_publisher(Bool, '/robot_0/Mode', 10)

        # Subscriber for control messages from the dashboard
        self.sub_control = self.create_subscription(String, '/control', self.key_callback, 10)

        # Send initial zero velocity
        self.send_initial_zero_velocity()

        self.autonomous = True
        self.robot_num = 0
        self.robot_max = 3

    def send_initial_zero_velocity(self):
        """Send zero velocity to the robot."""
        zero_twist = Twist()
        zero_twist.linear.x = 0.0
        zero_twist.angular.z = 0.0
        self.turtlebot_pub.publish(zero_twist)

    def key_callback(self, msg):
        """Handle keyboard input and publish velocity commands."""
        if msg.data == 'm':
            boolean_msg = Bool()
            self.autonomous = not self.autonomous
            boolean_msg.data = self.autonomous
            self.boolean_pub.publish(boolean_msg)
            return  # Exit function so Twist is not sent

        turtlebot_twist = Twist()

        if msg.data == 'w':  # Move forward
            turtlebot_twist.linear.x = 0.3
        elif msg.data == 's':  # Move backward
            turtlebot_twist.linear.x = -0.3
        elif msg.data == 'a':  # Turn left
            turtlebot_twist.angular.z = 1.0
        elif msg.data == 'd':  # Turn right
            turtlebot_twist.angular.z = -1.0
        elif msg.data == ' ':  # Pause (Stop movement)
            turtlebot_twist.linear.x = 0.0
            turtlebot_twist.angular.z = 0.0
            self.get_logger().info("Pause triggered.")
        elif msg.data == 'f':  # Emergency Stop
            turtlebot_twist.linear.x = 0.0
            turtlebot_twist.angular.z = 0.0
        elif msg.data == 'n':
            turtlebot_twist.linear.x = 0.0
            turtlebot_twist.angular.z = 0.0
            self.robot_num = (self.robot_num + 1) % self.robot_max
            self.turtlebot_pub = self.create_publisher(Twist, f'/robot_{self.robot_num}/HumanControl', 10)
            self.boolean_pub = self.create_publisher(Bool, f'/robot_{self.robot_num}/Mode', 10)
            ## 
            boolean_msg = Bool()
            self.autonomous = False
            boolean_msg.data = self.autonomous
            self.boolean_pub.publish(boolean_msg)


        # Publish the command
        self.turtlebot_pub.publish(turtlebot_twist)


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
