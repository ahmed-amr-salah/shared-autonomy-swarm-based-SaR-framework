import rospy
from nav_msgs.msg import Odometry

def test_callback(data):
    print('IMPLEMENT CALLBACK test_callback')
    middleware_test_pub.publish(data)

rospy.init_node('latency_test')


middleware_test_pub = rospy.Publisher('/middleware/test', Odometry, queue_size=10)


test_sub = rospy.Subscriber('/test',Odometry,test_callback)
            
if __name__ == '__main__':
    rospy.spin()