from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from subprocess import call
import socketio
import json
import re
import math
import message_converter
# from django.db import connections
import mysql.connector
import time
import uuid

# data = {"sensor_name": "Accelerometer1",
# "linear_acceleartion_x": 1,
#  "linear_accelaration_y": 1,
#  "linear_acceleration_z": 1,
#  "magnitude": 7}

# data = {"sensor_name": "Speed1",
#   "linear_x" : 2,
# "linear_y" : 3,
# "linear_z" : 4,
# "angular_x" : 1,
# "angular_y" : 1,
# "angular_z" : 1,
# "magnitude": 0.2}

# data = { "sensor_name": "Position",
# "position_x":5000,
# "position_y":2000,
# "position_z":3000
# }


#def dbInit(db_host,db_user,db_password,data_base,server_addr):
 #   server_addr = server_addr
  #  server_port = 8000
   # mydb = mysql.connector.connect(
   # host=db_host,
   # user= db_user,
   # password=db_password,
    #database=data_base,
    #autocommit=True,
    #port = 3306
    #)
   # cursor = mydb.cursor()
   # print('[INFO] Connecting to server http://{}:{}...'.format(server_addr, server_port))
    #return cursor, mydb

#def dbSave(data):
  #cursor, mydb = dbInit("172.20.10.4","root","root","velanalytics","0.0.0.0")

  # cursor = connections["default"].cursor()
 # sql_statement = f

def callback(data):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     #Send the data you got from sensors throgh ros node to the dashboard to update the graphs by emitting an event
  # sio.emit('sensedData', data ,namespace='/dashboard')
  data = message_converter.convert_ros_message_to_dictionary(data)

  data = json.loads(data['data'])
  

  data = preprocessing(data)
  print(data)
  #cursor, mydb = dbInit("172.20.10.4","root","root","velanalytics","0.0.0.0")

  # cursor = connections["default"].cursor()
  #if "Position" in data["sensor_name"]:
  #  sql_statement = f
 
#  sql_statement =
  #cursor.execute(sql_statement)
  obj = time.gmtime(0)
  epoch = time.asctime(obj)
  print("The time is: ", epoch)
  curr_time = round(time.time())
  print("Milliseconds since start: ", curr_time)

  for i in range(5):
    sio.emit('sensedData', dict(data) ,namespace='/dashboard')

def preprocessing(data):
    global i 
    i+=1
    print(i)
    data["time"] = json.dumps(datetime.now(), default=str)
    flag = False
    if(bool(re.search("^Accelerometer", data["sensor_name"]))):
      print("accelerometer")
      data = {"sensor_name" : data["sensor_name"],
            "magnitude" : data["magnitude"],
            "time": json.dumps(datetime.now(), default=str)
          }
      flag = True
      # dbSave(data)    

    elif(bool(re.search("^Speed", data["sensor_name"]))):
      print("speed")
      data = {"sensor_name" : data["sensor_name"],
              "magnitude" : data['magnitude'],
              "time": json.dumps(datetime.now(), default=str)
            }
      flag = True
      
    if (bool(re.search("^Temperature", data["sensor_name"]))):
      print("temperature")
      data = {"sensor_name" : data["sensor_name"],
              "magnitude" : data['magnitude'],
              "time": json.dumps(datetime.now(), default=str)
            }
      flag = True

    if(bool(re.search("^Pressure", data["sensor_name"]))):
      print("pressure")
      data = {"sensor_name" : data["sensor_name"],
              "magnitude" : data['magnitude'],
              "time": json.dumps(datetime.now(), default=str)
            }
      flag = True

    if(bool(re.search("^Position", data["sensor_name"]))):
      print("position")
      data = {"sensor_name" : data["sensor_name"],
              "lat": data['position_x'],
              "lng": data['position_y'],
              "time": json.dumps(datetime.now(), default=str)
            }
    else:  
      if(i==30):      
        #dbSave(data)
        i=0
    


        
    return data


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    unique_name = f"listener_{uuid.uuid4()}"
    rclpy.init()
    node = rclpy.create_node(unique_name)
    rclpy.spin(node)
    node.create_subscription(String, "/metric", callback)
    node.destroy_node()
    rclpy.shutdown()

sio = socketio.Client()


@sio.event(namespace='/dashboard')
def connect():
  print('[INFO] Successfully connected to server.')
  # for i in range(1000):
  # callback(data)
  # sio.emit('sensedData', "Hello!" ,namespace='/dashboard')


@sio.event(namespace='/dashboard')
def connect_error():
  print('[INFO] Failed to connect to server.')


@sio.event(namespace='/dashboard')
def disconnect():
  print('[INFO] Disconnected from server.')

#new data recieved
@sio.event(namespace='/dashboard')
def data_Changed(data):
  print(data)

if __name__ == '__main__':
    sio.connect('http://10.40.53.229:8000')
    global i 
    i=0
    listener()
