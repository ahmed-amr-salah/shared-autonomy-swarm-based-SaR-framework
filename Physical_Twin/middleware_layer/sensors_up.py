import rospy
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String
import json
import socketio
import os


sio = socketio.Client()

def position_cb(msg):
    print(msg)
    sio.emit('sensedPos', msg.data, namespace='/physicalSensors')



@sio.event(namespace='/physicalSensors')
def connect():
    print('Successfully connected to the server')

@sio.event(namespace='/physicalSensors')
def connect_error():
    print('Failed to connect to the server')

@sio.event(namespace='/physicalSensors')
def disconnect():
    print('Disconnected from the server')

def data_Changed(data):
    print(data)

if __name__ == '__main__':
    sio.connect(os.path.expandvars('http://$HOST_IP:8000/physicalSensors'))
    print(os.path.expandvars('http://$HOST_IP:8000'))
    rospy.init_node('sensorsMiddleware', anonymous=True)

    positionSub = rospy.Subscriber("positionData", String, position_cb)
    rospy.spin()