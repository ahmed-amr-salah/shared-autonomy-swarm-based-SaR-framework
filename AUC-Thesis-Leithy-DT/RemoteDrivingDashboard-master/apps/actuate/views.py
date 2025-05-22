# -*- coding: utf-8 -*-

from __future__ import unicode_literals
import socketio
import os

from django.shortcuts import redirect  # render ,
from django.template.response import TemplateResponse as render
from django.http import HttpResponse
from src.local_cloud.Query_Search import select_data, create_DB, actuate_DB
from src.local_cloud.qt_rcv import  rcv, actuateData
import json
from apps.home.views import sio


async_mode = None

basedir = os.path.dirname(os.path.realpath(__file__))
# sio = socketio.Server(async_mode='eventlet',cors_allowed_origins=['http://127.0.0.1:8000','http://localhost:8000','http://0.0.0.0:8000'])\

#web
##################################################################
@sio.on('digitalPos', namespace='/web')
def digitalPos(sid, message):
    print(message)
    sio.emit('position', message, namespace='/control')
##################################################################   
# @sio.on('new_data', namespace='/web')
# #Add the definition of your functions here as follows use the fn you created in Middleware
# def newData(sid, data):
#     sio.emit('Dataa', data, namespace='/trial1_namespace')

#Generated Code Region


@sio.on('latency_test_data', namespace='/dashboard')
#Add the definition of your functions here as follows use the fn you created in Middleware
def newData(sid, data):
    print(data)
    sio.emit('fn', data, namespace='/latency_test_namespace')



#End of Generated Region

@sio.on('leithy', namespace='/dashboard')
#Add the definition of your functions here as follows use the fn you created in Middleware
def newData(sid, data):
    print(data)
    sio.emit('fn', data, namespace='/hello_world_digital_twin_temp_namespace')

#TODO: Put the /physicalSensors namespace in the correct place
@sio.on('connect', namespace='/physicalSensors')
def connect_physicalSensors(sid, data):
    print('[INFO] Physical sensors connected: {}'.format(sid))

@sio.on('disconnect', namespace='/physicalSensors')
def disconnect_physicalSensors(sid):
    print('[INFO] Physical sensors disconnected: {}'.format(sid))

@sio.on('sensedPos', namespace='/physicalSensors')
def sensedPos(sid, data):
    sio.emit('sensedPos', data, namespace='/web')

@sio.on('Digital_sensors', namespace='/web')
def sensedData(sid,data):
    sio.emit('sensedData', data, namespace='/movement')


@sio.on('connect', namespace='/control')
def connect_control(sid, data):
    print('[INFO] Control connected: {}'.format(sid))

@sio.on('disconnect', namespace='/control')
def disconnect_control(sid):
    print('[INFO] Control disconnected: {}'.format(sid))

@sio.on('sensedPos', namespace='/control')
def sensedPos(sid, data):
    sio.emit('sensedPos', data, namespace='/web')

    
@sio.on('connect', namespace='/web')
def connect_web(sid, data):
    print('[INFO] Web client connected: {}'.format(sid))



@sio.on('disconnect', namespace='/web')
def disconnect_web(sid):
    print('[INFO] Web client disconnected: {}'.format(sid))


@sio.on('new_data', namespace='/web')
def newData(sid,data):
    sio.emit('actuateData',data,namespace='/movement')

#cv
@sio.on('connect', namespace='/cv')
def connect_cv(sid, data):
    print('[INFO] CV client connected: {}'.format(sid))



@sio.on('disconnect', namespace='/cv')
def disconnect_cv(sid):
    print('[INFO] CV client disconnected: {}'.format(sid))


@sio.on('cv2server',namespace='/cv')
def handle_cv_message(sid,message):
    sio.emit('server2web', message, namespace='/web')

#dashboard
@sio.on('connect', namespace='/dashboard')
def connect_dashboard(sid, data):
    print('[INFO] dashboard connected: {}'.format(sid))
    

@sio.on('disconnect', namespace='/dashboard')
def disconnect_dashboard(sid):
    print('[INFO] dashboard disconnected: {}'.format(sid))


@sio.on('sensedData',namespace='/dashboard')
def handle_sensed_data(sid,message):
    sio.emit('sensedData', message, namespace='/web')


@sio.on('connect', namespace='/dynamicDB')
def connect_QT(sid,data):
    print("Hi from".format(sid))
    print('[INFO] Create_DynamicDB connected: {}'.format(sid))




def main(request):
    return render(request, "home/dashboard_layout.html")
    


def actuate_data(request):

    if request.is_ajax():
        speed = request.POST.get('speed_control')
        print("_____________________",speed)
        # actuate_DB(speed, 'control_speed')
    return render(request, "actuate/actuate_data.html", {'actuateData': json.dumps(actuateData())})
#
def steering_angle(request):
    if request.is_ajax():
        angle = request.POST.get('steering_angle')
        print("_____________________",angle)
        # actuate_DB(angle, 'steering_angle')
    return render(request, "actuate/actuate_data.html")

def direction(request):
    if request.is_ajax():
        direction = request.POST.get('direction')
        print("_____________________",direction)
        # actuate_DB(direction, 'direction')
    return render(request, "actuate/actuate_data.html")



