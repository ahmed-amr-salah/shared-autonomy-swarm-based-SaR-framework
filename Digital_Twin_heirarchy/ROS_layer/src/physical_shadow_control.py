#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
#from pynput import keyboard as kb
from std_msgs.msg import String
import json
import signal
import sys



current_state = State()
global posee 
posee = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def control_position(msg):
    print(msg)
    global first_coordinate_saved, first_position
    data = json.loads(msg.data)
    positionX = data['position']['x']
    positionY = data['position']['y']
    positionZ = data['position']['z']

    if not first_coordinate_saved:
        first_position = [positionX, positionY, positionZ]
        first_coordinate_saved = True

    positionX -= first_position[0]
    positionY -= first_position[1]
    positionZ -= first_position[2]

    velocityX = data['velocity']['x']
    velocityY = data['velocity']['y']
    velocityZ = data['velocity']['z']
    accelerationX = data['acceleration']['x']
    accelerationY = data['acceleration']['y']
    accelerationZ = data['acceleration']['z']
    pose = PositionTarget()

    #8aleban dah mloosh lazma 

    pose.position.x = positionX
    pose.position.y = positionY
    pose.position.z = positionZ
    pose.velocity.x = velocityX
    pose.velocity.y = velocityY
    pose.velocity.z = velocityZ
    pose.acceleration_or_force.x = accelerationX
    pose.acceleration_or_force.y = accelerationY
    pose.acceleration_or_force.z = accelerationZ
    # hena el lazma kolaha
    posee.pose.position.x = positionX+10
    posee.pose.position.y = positionY+10
    posee.pose.position.z = positionZ+10
    pub.publish(posee)
       

if __name__ == "__main__":
    
    rospy.init_node("offb_node_py")
    state_sub = rospy.Subscriber("/mavros/state", State, callback=state_cb)
    middleware_sub = rospy.Subscriber("/middleware/control", String, control_position)
    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
        pub.publish(posee)
        print('Not Connected')
    print('Connected')

    arming_client(True)
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    arming_client.call(arm_cmd)
    set_mode_client(0,"AUTO.TAKEOFF")
    print ('Taking off.....\r')
    rospy.sleep(5)
    


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
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        pub.publish(posee)
        rate.sleep()
# 