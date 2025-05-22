import rospy
from nav_msgs.msg import Odometry
import time 
import socketio
import os

def middleware_test_callback(data):
    print('IMPLEMENT CALLBACK middleware_test_callback')


sio = socketio.Client()
@sio.event(namespace='/latency_test_namespace')
def connect(): 
    print('Successfully connected to the server')

@sio.event(namespace='/latency_test_namespace')
def connect_error(): 
    print('Failed to connect to the server')

@sio.event(namespace='/latency_test_namespace')
def disconnect(): 
    print('Disconnected from the server')

#Add the definition of your functions here as follows
@sio.event(namespace='/latency_test_namespace')
def fn(data):
    print('Seq:', data['header']['seq'], end= " ")
    print('Time:' , time.time())
    # print(data)

rospy.init_node('latency_test_mw')


middleware_test_sub = rospy.Subscriber('/middleware/test',Odometry,middleware_test_callback)
            

test_pub = rospy.Publisher('/test', Odometry, queue_size=10)


if __name__ == '__main__':
    sio.connect(os.path.expandvars('http://$HOST_IP:8000'))

    rospy.spin()