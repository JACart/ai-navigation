#!/usr/bin/env python

# import gps
# import speed
import socketio
import time
import rospy
import json
import vlc
import os
import numpy as np
import network_camera as camera
import base64
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int8, String, Bool
from geometry_msgs.msg import PoseStamped
from navigation_msgs.msg import GoalWaypoint, VehicleState, EmergencyStop, LatLongPoint
from sensor_msgs.msg import NavSatFix, Image
from rospy.numpy_msg import numpy_msg
isConnected = False

id = '1bcadd2fea88'

sio = socketio.Client()
empty = True

########################
### Recieving Events ###
########################f
@sio.event(namespace='/cart')
def connect():
    print('id: ', sio.sid)
    print('connection established')
    send('connect', id)
    isConnected = True

    
@sio.event(namespace='/cart')
def disconnect():
    #camera.cleanUp()
    isConnected = False
    print('disconnected from server')

def send(msg, data):
    sio.emit(msg, data, namespace='/cart')
    
@sio.on('cart_request', namespace='/cart')
def onCartRequest(data):
    lat_long = json.loads(data)
    print(str(data))
    msg = LatLongPoint()
    msg.latitude = lat_long["latitude"]
    msg.longitude = lat_long["longitude"]
    req_pub.publish(-1,21)
    #print("REQUEST: " + str(msg))
    #gps_request_pub.publish(msg)
    #Debug information
    #print("sending that we arrived after getting destination")
    #send("arrived", '/cart')

# @sio.on('cart_request', namespace='/cart')
# def onCartRequest(data):
#     lat_long = json.loads(data)
#     msg = LatLongPoint()
#     msg.latitude = lat_long["latitude"]
#     msg.longitude = lat_long["longitude"]
#     gps_request_pub.publish(msg)

@sio.on('destination', namespace='/cart')
def onDestination(msg):
    location_speech_pub.publish(False)
    safety_constant_pub.publish(True)

    print("RECIEVED GOAL: " + str(msg))
    location_string = str(msg)
    # Delete White Spaces
    location_string = location_string.replace(" ", "")
    # Lowercase Entire String
    location_string = location_string.lower()
    #Process the string into a waypoint
    calculated_waypoint = locationFinder(location_string)
    #Prepare goal waypoint message
    requested_waypoint = GoalWaypoint()
    requested_waypoint.start = -1
    requested_waypoint.goal = calculated_waypoint
    #Send requested waypoint to planner
    req_pub.publish(requested_waypoint)

@sio.on('pull_over',namespace='/cart')
def onPullOver():
    send_stop(True, 1)

@sio.on('resume_driving',namespace='/cart')
def onResume():
    send_stop(False, 1)
    
@sio.on('stop',namespace='/cart')
def onStop(data):
    send_stop(True, 1)
    
@sio.on('transit_await',namespace='/cart')
def onTransitAwait():
    print("TransitAwait")
    time.sleep(4)
    location_speech_pub.publish(True)
    

    
######################
### Sending Events ###
######################

def arrivedDestination():
    safety_exit_pub.publish(True)
    exit_sound.stop()
    exit_sound.play()
    print("arrived full")
    send('arrived','/cart')
    
def arrivedEmptyDestination():
    print("arrived empty")
    enter_sound.stop()
    enter_sound.play()
    #location_speech_pub.publish(True)
    send('arrived','/cart')

def send_audio(msg):
    data = {
        "msg": msg.data,
        "id":id
    }
    send('audio',json.dumps(data))
    
def send_video(msg, p_video):
    time.sleep(0.1)
    if(p_video == True):
        send('passenger_video', msg)
        #print("sent a image 1")
    else:
        send('cart_video', msg)
        #print("sent a image 2")
    
    
def sendLocation(msg):
    data = {
        'latitude': msg.latitude,
        'longitude': msg.longitude,
        'id': id
    }
    send('current_location',json.dumps(data))
    
def sendPassengerExit():
    send('passenger_exit',id)

def sendPositionIndex(data):
    send("position", data.data)

#pose tracking send when ready, AI start driving
def sendReady():
    send('passenger_ready',id)

#pose tracking send when unsafe
def sendUnsafe():
    send('passenger_unsafe',id)

#######################
### Other Functions ###
#######################
def locationFinder(location_string):
    if(location_string == "home"): #near the garage
        return 28
    if(location_string == "xlabs"): #front of xlabs
        return 23
    if(location_string == "cafeteria"): #the exit from xlabs
        return 12
    if(location_string == "clinic"): #right after the intersection heading towards the exit
        return 6
    if(location_string == "reccenter"): #on the straight away going away from the garage towards the front
        return 1
    return 28

        
def pullover_callback(msg):
    if msg.data == True:
        send_stop(True, 2)
        sendUnsafe()
    else:
        send_stop(False, 2)
        sendReady() #is this how it should work?'

def passenger_safe_callback(msg):
    if msg.data == True:
        send_stop(False, 3)
        sendReady()
    else:
        send_stop(True, 3)
        sendUnsafe()

def passenger_exit_callback(msg):
    sendPassengerExit()
    

#Handles destination arrival as well as various other vehicle state changes
def status_update(data):
    global empty
    if data.is_navigating == False:
        if data.reached_destination == True:
            if empty == True:
                empty = False
                arrivedEmptyDestination()
            else:
                empty = True
                arrivedDestination()
                
def update_image(msg):
    bridge = CvBridge() 
    image_raw = np.frombuffer(msg.data, dtype=np.uint8)
    image_raw.shape = (msg.height, msg.width, 3)
    image_raw = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
    h,w = image_raw.shape[:2]
    # Crop image and get the image width and height  
    cropped = image_raw[0:h, 0:672]  
    final_image = cv2.flip(cropped, -1)
    
    f2 = camera.rescale_frame(final_image, 20)
    retval, buffer = cv2.imencode('.jpg', f2)    
    send_video(base64.b64encode(buffer), True)
    send_video(camera.getVideo(), False)

# sender_id is important to ensure all parties 
# are ready to resume before releasing the stop command
# ie both voice and pose tell us we need to stop and then
# pose gives us the all clear but we should still be 
# waiting for voice to also give the all clear
# sender_id = 1 is the server, 2 is voice, 3 is pose, 4 is health monitor, 
# 0 is for internal usage but is currently unused
def send_stop(stop, sender_id):
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = stop
    stop_msg.sender_id = sender_id
    stop_pub.publish(stop_msg)

#####################
### Initialization ###
#####################
if __name__ == "__main__":
    rospy.init_node('network_node')
    stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
    req_pub = rospy.Publisher('/path_request', GoalWaypoint, queue_size=10)
    location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
    gps_request_pub = rospy.Publisher('/gps_request', LatLongPoint, queue_size=10)
    safety_constant_pub = rospy.Publisher('/safety_constant', Bool, queue_size=10)
    safety_exit_pub = rospy.Publisher('/safety_exit', Bool, queue_size=10)

    rospy.Subscriber('/camera/image_raw', Image, update_image)
    rospy.Subscriber('/current_position', Int8, sendPositionIndex)
    rospy.Subscriber('/vehicle_state', VehicleState, status_update)
    rospy.Subscriber('/pullover', Bool, pullover_callback)
    rospy.Subscriber('/speech_text', String, send_audio)
    rospy.Subscriber('/gps_coordinates', NavSatFix, sendLocation)
    rospy.Subscriber('/passenger_safe', Bool, passenger_safe_callback)
    rospy.Subscriber('/passenger_exit', Bool, passenger_exit_callback)
    
    exit_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "exit.mp3"))
    enter_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "enter.mp3"))
    rate = rospy.Rate(10)  # 10hz

    rospy.loginfo("Attempting connection with socketio server")
    sio.connect('http://35.238.125.238:8020', namespaces=['/cart'])#172.30.167.135
    while not rospy.is_shutdown():
        rate.sleep()


