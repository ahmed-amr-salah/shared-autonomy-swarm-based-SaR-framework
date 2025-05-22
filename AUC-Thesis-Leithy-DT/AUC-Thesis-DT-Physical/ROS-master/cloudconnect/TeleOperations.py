import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
import time
from std_msgs.msg import String



BURGER_MAX_LIN_VEL = 5.0
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

# key = ''
import threading
import time


import socketio


def move(key):
    global pub
    msg = String()
    msg.data = key
    pub.publish(msg)                                         


sio = socketio.Client()


@sio.event(namespace='/movement')
def connect():
    print('[INFO] Successfully connected to server.')

@sio.event(namespace='/movement')
def connect_error():
    print('[INFO] Failed to connect to server.')

@sio.event(namespace='/movement')
def disconnect():
    print('[INFO] Disconnected from server.')

@sio.event(namespace='/movement')
def actuateData(data):
    print(data)
    move(data)

# class T1 (threading.Thread):
#     def __init__(self):
#         super(T1 , self).__init__(name="T1 thread")
#         print('I am T1')

#     def run(self):
#         global key
#         global sio
#         # key = 'w'
#         # time.sleep(5)
#         # key='a'
#         @sio.event(namespace='/movement')
#         def actuateData(data):
#           global key
#           key=data

# t1 = T1()

# t1.start()




class CloudConnectMoce(Node):
    def __init__(self):
        super().__init__('cloud_connect_move')

        # Publisher for control
        global pub
        pub = self.create_publisher(String, '/control', 10)


# if __name__=="__main__": 
CURRENT_IP = os.getenv("HOST_IP")
URL = f'http://{CURRENT_IP}:8000'
sio.connect(URL)
# if __name__=="__main__": 

# sio.connect(os.path.expandvars('http://$HOST_IP:8000'))

rclpy.init()
node = CloudConnectMoce()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
