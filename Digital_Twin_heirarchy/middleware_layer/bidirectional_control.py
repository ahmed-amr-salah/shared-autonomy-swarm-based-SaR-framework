import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String
import json
import socketio
import os
import uuid

sio = socketio.Client()
@sio.event(namespace='/control')
def connect():
    print('Successfully connected to the server')

@sio.event(namespace='/control')
def connect_error():
    print('Failed to connect to the server')

@sio.event(namespace='/control')
def disconnect():
    print('Disconnected from the server')

@sio.event(namespace='/control')
def position(data):
    print(data)
    pub.publish(data)
    

if __name__ == '__main__':
    sio.connect(os.path.expandvars('http://$HOST_IP:8000/control'))
    print(os.path.expandvars('http://$HOST_IP:8000'))
    #rospy.init_node('sensorsMiddlewareDig', anonymous=True)

    unique_name = f"controlReceived_{uuid.uuid4()}"
    rclpy.init()
    node = rclpy.create_node(unique_name)
    rclpy.spin(node)
    pub = node.create_publisher(String, "/middleware/control", 1)
    node.destroy_node()
    rclpy.shutdown()