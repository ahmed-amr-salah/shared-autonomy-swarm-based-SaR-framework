import sys, os
from matplotlib import image
# import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import socketio
import base64
import time

import airsim
# import pprint
# import tempfile




    
sys.path.append('/usr/local/lib/python2.7/site-packages')
def image_callback():
    print("Received an image!")

    try:
    #   im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    #   print(type(im))
    #   print(im)
    #   frame = cv2.resize(im,(300,150), interpolation = cv2.INTER_AREA)

      while True:
            # Read a frame from the webcam
            responses = client.simGetImages([
            # airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
            airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
            airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
            airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
            print('Retrieved images: %d' % len(responses))


            for idx, response in enumerate(responses):



                if response.pixels_as_float:
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                    #airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
                elif response.compress: #png format
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                    #airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
                else: #uncompressed array
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
                    img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
                    img_rgb = cv2.resize(img_rgb,(300,150), interpolation = cv2.INTER_AREA)
                    streamer.send_data(img_rgb)

    except:
      print("error")      

class CVClient(object):
    def __init__(self, server_addr, stream_fps):
        self.server_addr = server_addr
        self.server_port = 8000
        self._stream_fps = stream_fps
        self._last_update_t = time.time()
        self._wait_t = (1/self._stream_fps)

    def _convert_image_to_jpeg(self, image):
        
        #frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Encode frame as jpeg
        frame = cv2.imencode('.jpg', image)[1].tobytes()
        # Encode frame in base64 representation and remove
        # utf-8 encoding
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        frame = base64.b64encode(frame).decode('utf-8')
        return "data:image/jpeg;base64,{}".format(frame)

    def send_data(self, frame):
        cur_t = time.time()
        if cur_t - self._last_update_t > self._wait_t:
            self._last_update_t = cur_t
            sio.emit(
                    'cv2server',
                    {
                        'image': self._convert_image_to_jpeg(frame),
                    },namespace='/cv')

    def close(self):
        sio.disconnect()


sio = socketio.Client()


def convert_image_to_jpeg(image):
        # Encode frame as jpeg
        frame = cv2.imencode('.jpg', image)[1].tobytes()
        # Encode frame in base64 representation and remove
        # utf-8 encoding
        frame = base64.b64encode(frame).decode('utf-8')
        return "data:image/jpeg;base64,{}".format(frame)

@sio.event(namespace='/cv')
def connect():
    global streamer
    streamer = CVClient('0.0.0.0', 5.0)
    print('[INFO] Successfully connected to server.')


@sio.event(namespace='/cv')
def connect_error():
    print('[INFO] Failed to connect to server.')


@sio.event(namespace='/cv')
def disconnect():
    print('[INFO] Disconnected from server.')
	
	
if __name__ == '__main__':
    sio.connect(os.path.expandvars('http://$HOST_IP:8000'))

    client = airsim.VehicleClient()
    image_callback()
