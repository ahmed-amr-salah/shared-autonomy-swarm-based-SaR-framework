#! /usr/bin/env python3

#imports
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import sys
import signal
from std_msgs.msg import String

#global variables
current_state = State()
global local_params
global attitude

#callbacks
def state_cb(msg):
    global current_state
    current_state = msg
def attitude_cb(msg):
    rospy.loginfo("Thrust: {}".format(msg.thrust))
#control functions
def increase_attitude():
    #increase thrust
    if attitude.thrust < 1:
        attitude.thrust += 0.1
def decrease_attitude():
    #increase thrust
    if attitude.thrust > 0:
        attitude.thrust -= 0.1
def right():
    local_params.position.x += 1
    attitude.orientation.y = 0.50
def left():
    local_params.position.x -= 1
    attitude.orientation.y = -0.50
def roll_right():
    attitude.orientation.x = 0.50
def roll_left():
    attitude.orientation.x = -0.50
def pitch_forward():
    attitude.orientation.y = 0.50
def pitch_backward():
    attitude.orientation.y = -0.50
def yaw_right():
    attitude.orientation.z = 0.50
def yaw_left():
    attitude.orientation.z = -0.50
def on_press(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    if msg.data == "w":
        #increase altitude using thrust
        increase_attitude()
    elif msg.data == "s":
        decrease_attitude()
        #decrease altitude using thrust
    elif msg.data == "a":
        #move left
        left()
    elif msg.data == "d":
        #move right
        right()
    elif msg.data == "q":
        roll_left()
    elif msg.data == "e":
        roll_right()
    elif msg.data == "r":
        pitch_forward()
    elif msg.data == "f":
        pitch_backward()
    elif msg.data == "t":
        yaw_right()
    elif msg.data == "g":
        yaw_left()


def signal_handler(sig, frame):
    rospy.loginfo('KeyboardInterrupt (ID: {}) has been caught. Cleaning up...'.format(sig))
    set_manual_mode()
    sys.exit(0)

def set_manual_mode():
    manual_set_mode = SetModeRequest()
    manual_set_mode.custom_mode = 'MANUAL'
    try:
        if(set_mode_client.call(manual_set_mode).mode_sent):
            rospy.loginfo("Manual mode enabled")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    #initialize the node
    rospy.init_node("offb_node_py")
    
    #subscribers
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    set_attitude_sub = rospy.Subscriber("mavros/setpoint_raw/target_attitude", AttitudeTarget, attitude_cb)
    #middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)
    middleware_sub = rospy.Subscriber("/middleware/control", String, on_press)
    #publishers
    #this sends the throttle to the drone through the topic that mavros is subsribed to
    set_attitude_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    stp_local_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

    #services
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode) 

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    #/mavros/setpoint_raw/local deals with vel, acc, position
    local_params = PositionTarget()
    local_params.header.stamp = rospy.Time.now()
    local_params.velocity.x = 0
    local_params.velocity.y=0
    local_params.velocity.z=0

    #/mavros/setpoint_raw/attitude deals with thrust and orientation
    attitude = AttitudeTarget()
    attitude.thrust = 0.3
    #Modify positions for the drone to go right, left, up and down
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 1
    pose.pose.position.z = 0

    #KeyboardInterrupt
    signal.signal(signal.SIGINT, signal_handler)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break
        #publish attitude set points
        set_attitude_pub.publish(attitude)
        #Publish local set points
        stp_local_pub.publish(local_params)
        local_pos_pub.publish(pose)
        rate.sleep()

    # Set the mode to AUTO.TAKEOFF
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Arm the vehicle
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if (current_state.mode != "MANUAL" and current_state.mode != "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                    set_attitude_pub.publish(attitude)
                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

        local_pos_pub.publish(pose)
        stp_local_pub.publish(local_params)
        set_attitude_pub.publish(attitude)
        rate.sleep()   
