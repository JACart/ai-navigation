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
from navigation_msgs.msg import GoalWaypoint, EmergencyStop, VehicleState
from speech_recognition_location import startRecognize,stopRecognize
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

def sendAudio(msg):
    json = {
        "msg": msg,
        "id":id
    }
    send('audio',JSON.dumps(json))

#audio call pullover
def sendPullOver():
   send('pull_over',id)
   
#pose tracking send when unsafe
def sendUnsafe():
    send('passenger_unsafe',id)
    

def sendPassengerExit():
    send('passenger_exit',id)
    
#pose tracking send when ready, AI start driving
def sendReady():
    send('passenger_ready',id)

@sio.on('pull_over',namespace='/cart')
def onPullOver():
    print("help this needs changed")
    #do stuff for the AI

@sio.on('resume_driving',namespace='/cart')
def onResume():
    print("help this needs changed")
    #do stuff for the AI

#send index + lat/lng + string
@sio.on('destination', namespace='/cart')
def onDestination(data):
    #Get JSON data
    location_speech_pub.publish(False)
    #stopRecognize()
   ## {latitude:123, longtidue:435}
    raw_waypoint = data

    #Prepare goal waypoint message
    requested_waypoint = GoalWaypoint()
    requested_waypoint.start = -1
    requested_waypoint.goal = raw_waypoint

    #Send requested waypoint to planner
    req_pub.publish(requested_waypoint)


@sio.on('stop',namespace='/cart')
def onStop(data):
    stop_cart = EmergencyStop()
    stop_cart.emergency_stop = True
    stop_pub.publish(stop_cart)

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
            
def pulloverCallback(data):
    if data:
        sendPullOver()
    else:
        pass
        #sendReady() #is this how it should work?

if __name__ == "__main__":
    rospy.init_node('network_node')
    stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
    req_pub = rospy.Publisher('/path_request', GoalWaypoint, queue_size=10)
    location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
    #pub = rospy.Publisher('network_node_pub', String, queue_size=10)
    rospy.Subscriber('/current_position', Int8, sendPositionIndex)
    rospy.Subscriber('/vehicle_state', VehicleState, status_update)
    rospy.Subscriber('/pullover', Bool, pulloverCallback)
    rospy.Subscriber('/speech_text', String, sendAudio)
    rate = rospy.Rate(10)  # 10hz

    #rospy.spin()
    rospy.loginfo("Attempting connection with socketio server")
    sio.connect('http://35.238.125.238:8020', namespaces=['/cart'])
    while not rospy.is_shutdown():
        rate.sleep()


