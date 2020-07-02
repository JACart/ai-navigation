#!/usr/bin/env python

import socketio
import rospy
import time
from navigation_msgs.msg import EmergencyStop, LatLongPoint
from std_msgs.msg import String, Bool


"""
This file contains the code needed to communicate
with the actually server. It is called by the ros node
network node.
"""

id = '1bcadd2fea88'

sio = socketio.Client()

is_connected = False


    
########################
### Recieving Events ###
########################
@sio.event(namespace='/cart')
def connect():
    rospy.loginfo('connection established')
    send('connect', id)
    is_connected = True

        
@sio.event(namespace='/cart')
def disconnect():
    #camera.cleanUp()
    is_connected = False
    rospy.loginfo('disconnected from server')

def send(msg, data):
    sio.emit(msg, data, namespace='/cart')
    
@sio.on('cart_request', namespace='/cart')
def on_cart_req(data):
    lat_long = json.loads(data)
    rospy.loginfo("Latitude/Long received: " + str(data))
    msg = LatLongPoint()
    msg.latitude = lat_long["latitude"]
    msg.longitude = lat_long["longitude"]
    gps_request_pub.publish(msg)


@sio.on('destination', namespace='/cart')
def on_dest(msg):
    location_speech_pub.publish(False)
    safety_constant_pub.publish(True)

    rospy.loginfo("RECIEVED GOAL: " + str(msg))
    location_string = str(msg)
    # Delete White Spaces
    location_string = location_string.replace(" ", "")
    # Lowercase Entire String
    location_string = location_string.lower()
    #Process the string into a waypoint
    calculated_waypoint = locationFinder(location_string)
    #Send requested waypoint to planner
    req_pub.publish(calculated_waypoint)

@sio.on('pull_over',namespace='/cart')
def on_pull_over():
    rospy.loginfo("Recieved Pull Over")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = True
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)

@sio.on('resume_driving',namespace='/cart')
def onResume():
    rospy.loginfo("Received a resume signal")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = False
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)
    
@sio.on('stop',namespace='/cart')
def onStop(data):
    rospy.loginfo("Received a stop signal")
    stop_msg = EmergencyStop()
    stop_msg.emergency_stop = True
    stop_msg.sender_id = 1
    stop_pub.publish(stop_msg)
    
@sio.on('transit_await',namespace='/cart')
def onTransitAwait():
    rospy.loginfo("TransitAwait")
    time.sleep(4)
    location_speech_pub.publish(True)
    
if __name__ == "__main__":
    req_pub = rospy.Publisher('/destination_request', String, queue_size=10)
    gps_request_pub = rospy.Publisher('/gps_request', LatLongPoint, queue_size=10)
    location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
    stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
    
    rospy.loginfo("Attempting connection with socketio server")    
    sio.connect('http://35.238.125.238:8020', namespaces=['/cart'])#172.30.167.135
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()

    