#!/usr/bin/env python

import socketio
import time
import threading
import rospy
import json
import vlc
import os
import numpy as np
import base64
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int8, UInt64, String, Bool
from geometry_msgs.msg import PoseStamped
from navigation_msgs.msg import GoalWaypoint, VehicleState, EmergencyStop, LatLongPoint
from sensor_msgs.msg import NavSatFix, Image
from rospy.numpy_msg import numpy_msg

"""
This file contains the ros node: network node needed to be able 
to send the correct messages to the server. It talks to
network_client.
"""


        
empty = True
is_connected = False

id = '1bcadd2fea88'
sio = socketio.Client()
        
########################
### Recieving Events ###
########################
@sio.event(namespace='/ros')
def connect():
    rospy.loginfo('connection established')
    send('connect', id)
    is_connected = True

        
@sio.event(namespace='/ros')
def disconnect():
    is_connected = False
    rospy.loginfo('disconnected from server')

def send(msg, data):
    server_lock.acquire()
    try:
        sio.emit(msg, data, namespace='/ros')
    except:
        rospy.loginfo('Was unable to send data to the server')
    server_lock.release()
    
@sio.on('drive-to', namespace='/ros')
def on_cart_req(data):
    lat_long = json.loads(data)
    rospy.loginfo("Latitude/Long received: " + str(data))
    msg = LatLongPoint()
    msg.latitude = lat_long["latitude"]
    msg.longitude = lat_long["longitude"]
    gps_request_pub.publish(msg)


@sio.on('destination', namespace='/ros')
def on_dest(data):
    lat_long = json.loads(data)
    rospy.loginfo("Latitude/Long received: " + str(data))
    msg = LatLongPoint()
    msg.latitude = lat_long["latitude"]
    msg.longitude = lat_long["longitude"]
    gps_request_pub.publish(msg)

@sio.on('pull-over',namespace='/ros')
def on_pull_over():
    rospy.loginfo("Recieved Pull Over")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = True
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)

@sio.on('resume-driving',namespace='/ros')
def on_resume():
    rospy.loginfo("Received a resume signal")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = False
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)
    
@sio.on('stop',namespace='/ros')
def on_stop(data):
    rospy.loginfo("Received a stop signal")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = True
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)    
 
    
@sio.on('transit-await',namespace='/ros')
def on_transit_await():
    rospy.loginfo("TransitAwait")
    time.sleep(4)
    location_speech_pub.publish(True)
  
        
        
######################
### Sending Events ###
######################

def arrived_dest():
    safety_exit_pub.publish(True)
    send('arrived','/ros')
    
def arrived_empty_dest():
    send('arrived','/ros')

def send_audio(msg):
    data = {
        "msg": msg.data,
        "id":id
    }
    send('audio',json.dumps(data))
    
    
def send_location(msg):
    data = {
        'latitude': msg.latitude,
        'longitude': msg.longitude,
        'id': id
    }
    send('gps',json.dumps(data))
    
def send_passenger_exit():
    send('passenger-exit',id)

def send_position_index(data):
    send("position", data.data)

#pose tracking send when ready, AI start driving
def send_ready():
    send('transit-start',id)

#pose tracking send when unsafe
def send_unsafe():
    send('passenger-unsafe',id)
    


# sender_id is important to ensure all parties 
# are ready to resume before releasing the stop command
# ie both voice and pose tell us we need to stop and then
# pose gives us the all clear but we should still be 
# waiting for voice to also give the all clear
# sender_id = 1 is the server, 2 is voice, 3 is pose, 4 is health monitor, 
# 0 is for internal usage but is currently unused
def pullover_callback(msg):
    if msg.data:
        stop_msg = EmergencyStop()
        stop_msg.emergency_stop = True
        stop_msg.sender_id = 2
        stop_pub.publish(stop_msg)
        send_unsafe()
    else:
        send_ready()

def passenger_safe_callback(msg):
    if msg.data:
        send_ready()
    else:
        stop_msg = EmergencyStop()
        stop_msg.emergency_stop = True
        stop_msg.sender_id = 2
        stop_pub.publish(stop_msg)
        send_unsafe()

def passenger_exit_callback(msg):
    send_passenger_exit()
    

#Handles destination arrival as well as various other vehicle state changes
def status_update(data):
    if not data.is_navigating:
        if data.reached_destination:
            arrived_dest()
                
#Processes and sends the image from the zed camera

last_front_pub = -1
last_passenger_pub = -1

def passenger_image_callback(img_msg):
    global last_passenger_pub
    cur_time = time.time()
    if cur_time > last_passenger_pub + 1:
        bridge = CvBridge() 
        image_raw = np.frombuffer(img_msg.data, dtype=np.uint8)
        image_raw.shape = (img_msg.height, img_msg.width, 3)
        image_raw = bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        h,w = image_raw.shape[:2]
        # Crop image and get the image width and height  
        cropped = image_raw[0:h, 0:672]  
        final_image = cv2.flip(cropped, -1)
        dim = (400, 400)
        f2 = cv2.resize(final_image, dim, interpolation=cv2.INTER_AREA)
        retval, buffer = cv2.imencode('.jpg', f2)    
        send('passenger-video',base64.b64encode(buffer) )
        last_passenger_pub = cur_time
    


def front_image_callback(img_msg):
    global last_front_pub
    cur_time = time.time()
    if cur_time > last_front_pub + 1:
        cur_time = time.time()
        bridge = CvBridge() 
        image_raw = bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        retval, buffer = cv2.imencode('.jpg', image_raw)    
        send('cart-video', base64.b64encode(buffer))
        last_front_pub = cur_time

def eta_callback(msg):
    send('arrived-time', msg.data)

if __name__ == "__main__":
    rospy.init_node('network_node')
    server_lock = threading.Lock()
    
    try:
        rospy.loginfo("Attempting connection with socketio server")    
        sio.connect('http://localhost:8021', namespaces=['/ros'])#172.30.167.135
    except:
        rospy.loginfo('Was unable to connect to the server')
    
    stop_pub = rospy.Publisher('/stop', EmergencyStop, queue_size=10)
    req_pub = rospy.Publisher('/destination_request', String, queue_size=10)
    location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
    gps_request_pub = rospy.Publisher('/gps_request', LatLongPoint, queue_size=10)
    safety_constant_pub = rospy.Publisher('/safety_constant', Bool, queue_size=10)
    safety_exit_pub = rospy.Publisher('/safety_exit', Bool, queue_size=10)
    

    rospy.Subscriber('/zed/image_raw', Image, passenger_image_callback)
    rospy.Subscriber('/front_facing/image_raw', Image, front_image_callback)
    rospy.Subscriber('/current_position', Int8, send_position_index)
    rospy.Subscriber('/vehicle_state', VehicleState, status_update)
    rospy.Subscriber('/pullover', Bool, pullover_callback)
    rospy.Subscriber('/speech_text', String, send_audio)
    rospy.Subscriber('/gps_coordinates', NavSatFix, send_location)
    rospy.Subscriber('/passenger_safe', Bool, passenger_safe_callback)
    rospy.Subscriber('/passenger_exit', Bool, passenger_exit_callback)
    rospy.Subscriber('/eta', UInt64, eta_callback)
    
    exit_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "exit.mp3"))
    enter_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "enter.mp3"))
    
    rospy.spin()
        