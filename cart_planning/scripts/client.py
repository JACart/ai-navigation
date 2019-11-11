#!/usr/bin/env python

# import gps
# import speed
import socketio
import time
import rospy
import json
# import camera
# import destination
from std_msgs.msg import Int8, String
from geometry_msgs.msg import PoseStamped
from navigation_msgs.msg import GoalWaypoint, EmergencyStop

isConnected = False

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

#send index + lat/lng + string
@sio.on('destination', namespace='/cart')
def onDestination(data):
    #Get JSON data
    raw_data = json.loads(data)
    raw_waypoint = raw_data["index"]

    #Prepare goal waypoint message
    requested_waypoint = GoalWaypoint
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
    camera.cleanUp()
    isConnected = False
    print('disconnected from server')

def sendPositionIndex(data):
    send("position", data.data)

def arrivedDestination(data):
	send('arrived','','/cart')

rospy.init_node('network_node')
stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
req_pub = rospy.Publisher('/path_request', GoalWaypoint, queue_size=10)
#pub = rospy.Publisher('network_node_pub', String, queue_size=10)
rospy.Subscriber('/current_position', Int8, sendPositionIndex)
rate = rospy.Rate(10)  # 10hz

#rospy.spin()
rospy.loginfo("Attempting connection with socketio server")
sio.connect('http://172.30.172.30:8020', namespaces=['/cart'])
sio.wait()


