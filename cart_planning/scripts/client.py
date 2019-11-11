#!/usr/bin/env python

# import gps
# import speed
import socketio
import time
import rospy
# import camera
# import destination
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped

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
    rospy.loginfo(data)
    pub.publish(data)


@sio.on('stop',namespace='/cart')
def onStop(data):
    rospy.logInfo(data)
    pub.publish(data)

@sio.event(namespace='/cart')
def disconnect():
    camera.cleanUp()
    isConnected = False
    print('disconnected from server')

def sendPositionIndex(data):
    print(data)
    rospy.loginfo(data)
    send("position", data.pose.position.x)

def arrivedDestination(data):
	send('arrived','','/cart')

rospy.init_node('network_node')
pub = rospy.Publisher('network_node_pub', String, queue_size=10)
rospy.Subscriber('/ndt_pose', PoseStamped, sendPositionIndex)
rate = rospy.Rate(10)  # 10hz

sio.connect('http://172.30.172.30:8020', namespaces=['/cart'])
sio.wait()


