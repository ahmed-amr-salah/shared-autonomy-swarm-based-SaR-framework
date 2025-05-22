import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

def echo(msg):
    print(f'x: {msg.pose.pose.position.x} y: {msg.pose.pose.position.y} timestamp: {time.time()}')
    


rclpy.init()
node = rclpy.create_node('odom_echo')
rclpy.spin(node)
node.create_subscription(Odometry, "/odom", echo)
node.destroy_node()
rclpy.shutdown()