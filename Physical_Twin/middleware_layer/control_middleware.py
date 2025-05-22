from datetime import datetime
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sys, select, os
import socketio


def control_Drone(key):
    global pub
    pub.publish(key)

    print(key)

sio = socketio.Client()

@sio.event(namespace='/move')
def connect():
    print("Succcessfully connected to server")


@sio.event(namespace='/move')
def connect_error(err):
    print(f"Failed to connect to server: {err}")
    # Handle the connection error or perform any necessary actions here


@sio.event(namespace='/move')
def disconnect():
    print("Disconnected from server")

@sio.event(namespace='/movement')
def actuateData(data):
    print(data)
    control_Drone(data)



# In ROS, nodes are uniquely named. If two nodes with the same
# name are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
sio.connect(os.path.expandvars('http://$HOST_IP:8000'))
rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher("/middleware/control", String, queue_size=1)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
sio.wait()