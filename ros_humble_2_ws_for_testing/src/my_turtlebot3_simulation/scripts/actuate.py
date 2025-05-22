import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
from std_msgs.msg import String

BURGER_MAX_LIN_VEL = 5.0
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

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
    else:
        input = input
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
    else:
        vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel

status = 0
target_linear_vel = 0.0
target_angular_vel = 0.0
control_linear_vel = 0.0
control_angular_vel = 0.0

def c_move(key, node):
    global status, target_linear_vel, target_angular_vel, control_angular_vel, control_linear_vel
    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
    twist.linear.x = control_linear_vel
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = control_angular_vel

    # Publish the twist to all robots
    node.publish_twist_to_all(twist)

def move(key, node):
    global status, target_linear_vel, target_angular_vel, control_angular_vel, control_linear_vel

    if key == 'w':
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE, node.turtlebot3_model)
        status += 1
    elif key == 'x':
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE, node.turtlebot3_model)
        status += 1
    elif key == 'a':
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE, node.turtlebot3_model)
        status += 1
    elif key == 'd':
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE, node.turtlebot3_model)
        status += 1
    elif key == ' ' or key == 's':
        target_linear_vel = 0.0
        control_linear_vel = 0.0
        target_angular_vel = 0.0
        control_angular_vel = 0.0
    key = ''

    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE / 2.0))
    twist.linear.x = control_linear_vel
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE / 2.0))
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = control_angular_vel

    # Publish the twist to all robots
    node.publish_twist_to_all(twist)

def key_callback(msg, node):
    move(msg.data, node)

class Actuate(Node):
    def __init__(self):
        super().__init__('actuate_node')

        # Publishers for all robots' /cmd_vel topics
        self.turtlebot1_pub = self.create_publisher(Twist, '/turtlebot1_ns/cmd_vel', 10)
        self.turtlebot2_pub = self.create_publisher(Twist, '/turtlebot2_ns/cmd_vel', 10)
        # Add more publishers for additional robots here

        # Subscriber for control messages
        self.sub_control = self.create_subscription(String, '/control', lambda msg: key_callback(msg, self), 10)

        # Retrieve the turtlebot model parameter
        self.declare_parameter('model', 'burger')
        self.turtlebot3_model = self.get_parameter('model').get_parameter_value().string_value

    def publish_twist_to_all(self, twist):
        # Publish the twist to all robots
        self.turtlebot1_pub.publish(twist)
        self.turtlebot2_pub.publish(twist)
        # Add more publishers for additional robots here

rclpy.init()
node = Actuate()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
