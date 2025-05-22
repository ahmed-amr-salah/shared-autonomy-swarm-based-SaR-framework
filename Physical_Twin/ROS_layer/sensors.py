#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget 
from std_msgs.msg import String
import json

# def motion_cb(msg):
#     print(msg)
#     pos_x_coordinate = msg.position.x
#     pos_y_coordinate = msg.position.y
#     pos_z_coordinate = msg.position.z
#     acc_x_coordinate = msg.acceleration_or_force.x
#     acc_y_coordinate = msg.acceleration_or_force.y
#     acc_z_coordinate = msg.acceleration_or_force.z
#     vel_x_coordinate = msg.velocity.x
#     vel_y_coordinate = msg.velocity.y
#     vel_z_coordinate = msg.velocity.z
#     #encopass the three coordinates in a dictionary
#     motionData = {"position": {"x": pos_x_coordinate, "y": pos_y_coordinate, "z": pos_z_coordinate}, 
#                   "acceleration": {"x": acc_x_coordinate, "y": acc_y_coordinate, "z": acc_z_coordinate}, 
#                   "velocity": {"x": vel_x_coordinate, "y": vel_y_coordinate, "z": vel_z_coordinate}}
#     print("The data is",motionData)
#     motionData_json = json.dumps(motionData)
#     motionPub.publish(motionData_json)
    # pos_x_coordinate = msg.position.x
    # pos_y_coordinate = msg.position.y
    # pos_z_coordinate = msg.position.z
    # #encopass the three coordinates in a dictionary
    # positionData = {"x": pos_x_coordinate, "y": pos_y_coordinate, "z": z_coordinate}
    # print("The data is",positionData)
    
    # position_json = json.dumps(positionData)
    # print("The data in json format is",position_json)
    # postionPub.publish(position_json)
    #publish the data
    #postionPub.publish(positionData)

def motion_cb(msg):
    #want to get the relative position every time the position is published
    pos_x_coordinate = msg.pose.position.x
    pos_y_coordinate = msg.pose.position.y
    pos_z_coordinate = msg.pose.position.z
    #encopass the three coordinates in a dictionary
    motionData = {"position": {"x": pos_x_coordinate, "y": pos_y_coordinate, "z": pos_z_coordinate}}
    print("The data is",motionData)
    motionData_json = json.dumps(motionData)
    motionPub.publish(motionData_json)

if __name__ == '__main__':
    rospy.init_node('sensors', anonymous=True)
    # TODO: Subscribe to the 
    motionSub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, motion_cb)
    motionPub = rospy.Publisher("positionData", String, queue_size=10)
    rospy.spin()