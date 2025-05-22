#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
#from pynput import keyboard as kb
from std_msgs.msg import String
import signal
import sys


current_state = State()
current_position = PoseStamped()
def state_cb(msg):
    global current_state
    current_state = msg

def position_cb(msg):
    global current_position 
    current_position = msg

def set_manual_mode():
    manual_set_mode = SetModeRequest()
    manual_set_mode.custom_mode = 'MANUAL'
    try:
        resp = set_mode_client.call(manual_set_mode)
        if resp.mode_sent:
            rospy.loginfo("Manual Mode enabled")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s",e)


def signal_handler(sig, frame):
    rospy.loginfo("KeyboardInterrupt (ID: {}) has been caught. Changing to manual mode".format(sig))
    set_manual_mode()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

def increase_altitude():
    global pose
    pose.pose.position.z += 1
    local_pos_pub.publish(pose)


def decrease_altitude():
    global pose
    pose.pose.position.z -= 1
    local_pos_pub.publish(pose)

def move_left():
    global pose
    pose.pose.position.x -= 1
    local_pos_pub.publish(pose)

def move_right():
    global pose
    pose.pose.position.x += 1
    local_pos_pub.publish(pose)


def on_press(key):
    print(key)
    try:
        if key.data == 'w':
            print('w')
            increase_altitude()
        elif key.data == 'x':
            decrease_altitude()
        elif key.data == 'a':
            move_left()
        elif key.data == 'd':
            move_right()
    except AttributeError:
        pass #Handle special keys here if necessary

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("offb_node_py")

    signal.signal(signal.SIGINT, signal_handler)

    state_sub = node.create_subscriber(State, "/mavros/state", state_cb, QoSProfile(depth=10))
    local_pos_pub = node.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10, QoSProfile(depth=10))
    local_pos_sub = node.create_subscriber(PoseStamped, "/mavros/setpoint_position/local", position_cb, QoSProfile(depth=10))


    arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')
    while not arming_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /mavros/cmd/arming not available, waiting again...')

    set_mode_client = node.create_client(SetMode, '/mavros/set_mode')
    while not set_mode_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /mavros/set_mode not available, waiting again...')


    middleware_sub = node.create_subscriber(String, "/middleware/control", on_press, QoSProfile(depth=10))



    rate = node.create_rate(20)

    while not rclpy.ok() and not current_state.connected:
        rate.sleep()
        node.get_logger().info('Not Connected')
    node.get_logger().info('Connected')

    # arming_client(True)
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    arm_future = arming_client.call_async(arm_cmd)

    rclpy.spin_until_future_complete(node, arm_future)

    #check is there is a race condition because of spina nd spin_until_future_complete
    if arm_future.result() is not None:
        node.get_logger().info('Arming result: {}'.format(arm_future.result().success))
    else:
        node.get_logger().error('Arming service call failed.')

    set_mode_client(0,"AUTO.TAKEOFF")
    print ('Taking off.....\r')
    rospy.sleep(5)

    #current_position = PoseStamped()
    pose = PoseStamped()
    pose.pose.position.x = current_position.pose.position.x
    pose.pose.position.y = current_position.pose.position.y
    pose.pose.position.z = current_position.pose.position.z

    
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    # set_mode_client(0,"OFFBOARD")
    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'
    # set_mode_client.call(offb_set_mode)


    # # Create a listener for keyboard input
    # listener = kb.Listener(on_press=on_press)
    # listener.start()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()
    
    while not rospy.is_shutdown():
        while not current_state.armed:
            if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
            rate.sleep()
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        # else:
        #     if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
        #         if arming_client.call(arm_cmd).success:
        #             rospy.loginfo("Vehicle armed")
        #         last_req = rospy.Time.now()
        
        local_pos_pub.publish(pose)
        rate.sleep()
    #rospy.spin()