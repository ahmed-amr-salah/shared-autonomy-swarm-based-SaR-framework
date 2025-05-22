import cv2
import rclpy
from rclpy.node import Node
from matplotlib import image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


bridge = CvBridge()

rclpy.init()
node = rclpy.create_node('scan_images')
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

def image_publisher():

    try:
        cap = cv2.VideoCapture(0)  # 0 represents the default webcam, change it if necessary
        
        # Check if the webcam is opened correctly
        if not cap.isOpened():
            print("Cannot open webcam")
            exit()

        while True:
            # Read a frame from the webcam
            ret, frame = cap.read()
            print(frame)
            # Check if the frame was read successfully

            pub = node.create_publisher(Image, "/images", 10)
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(ros_image)
            print("Sent an image!")


        # Release the video capture device and close the window
     
    except:
        print('Err')

image_publisher()