#!/usr/bin/env python

# import gps
# import speed
import socketio
import time
import rospy
import json
# import camera
# import destination
from std_msgs.msg import Int8, String, Bool
from geometry_msgs.msg import PoseStamped
from navigation_msgs.msg import GoalWaypoint, VehicleState, EmergencyStop
from speech_recognition_location import startRecognize,stopRecognize
from sensor_msgs.msg import NavSatFix
isConnected = False

id = '1bcadd2fea88'

sio = socketio.Client()



@sio.event(namespace='/cart')
def connect():
    print('id: ', sio.sid)
    print('connection established')
    send('connect', '23423')

    
	
    isConnected = True
    # while isConnected:
    #     send('video', camera.getVideo())
    #     time.sleep(.1)


def send(msg, data):
    sio.emit(msg, data, namespace='/cart')

@sio.on('transit_await',namespace='/cart')
def onTransitAwait(data):
    location_speech_pub.publish(True)
    #startRecognize(sendAudio)

def send_audio(msg):
    json = {
        "msg": msg,
        "id":id
    }
    send('audio',JSON.dumps(json))

   
#pose tracking send when unsafe
def sendUnsafe():
    send('passenger_unsafe',id)
    
def sendPassengerExit():
    send('passenger_exit',id)
    
#pose tracking send when ready, AI start driving
def sendReady():
    send('passenger_ready',id)

def sendLocation(self):
    send('current_location',id)
    
@sio.on('pull_over',namespace='/cart')
def onPullOver():
    send_stop(True, 1)

@sio.on('resume_driving',namespace='/cart')
def onResume():
    send_stop(False, 1)

#send index + lat/lng + string
@sio.on('destination', namespace='/cart')
def onDestination(data):
    #Get JSON data
    location_speech_pub.publish(False)
    
    #{latitude:123, longtidue:435}
    raw_waypoint = data
    
    #Process the lat long into a waypoint
    calculated_waypoint = 0
    
    #Prepare goal waypoint message
    requested_waypoint = GoalWaypoint()
    requested_waypoint.start = -1
    requested_waypoint.goal = calculated_waypoint

    #Send requested waypoint to planner
    req_pub.publish(requested_waypoint)


@sio.on('stop',namespace='/cart')
def onStop(data):
    send_stop(True, 1)

@sio.event(namespace='/cart')
def disconnect():
    #camera.cleanUp()
    isConnected = False
    print('disconnected from server')

def sendPositionIndex(data):
    send("position", data.data)

def arrivedDestination(data):
	send('arrived','','/cart')

#Handles destination arrival as well as various other vehicle state changes
def status_update(data):
    if data.is_navigating == False:
        if data.reached_destination == True:
            send("arrived", '/cart')
            
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

if __name__ == "__main__":
    rospy.init_node('network_node')
    stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
    req_pub = rospy.Publisher('/path_request', GoalWaypoint, queue_size=10)
    location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
    #pub = rospy.Publisher('network_node_pub', String, queue_size=10)
    rospy.Subscriber('/current_position', Int8, sendPositionIndex)
    rospy.Subscriber('/vehicle_state', VehicleState, status_update)
    rospy.Subscriber('/pullover', Bool, pullover_callback)
    rospy.Subscriber('/speech_text', String, send_audio)
    rospy.Subscriber('/gps_coordinates', NavSatFix, sendLocation)
    rospy.Subscriber('/passenger_safe', Bool, passenger_safe_callback)
    rospy.Subscriber('/passenger_exit', Bool, passenger_exit_callback)
    rate = rospy.Rate(10)  # 10hz

    #rospy.spin()
    rospy.loginfo("Attempting connection with socketio server")
    sio.connect('http://35.238.125.238:8020', namespaces=['/cart'])
    while not rospy.is_shutdown():
        rate.sleep()


