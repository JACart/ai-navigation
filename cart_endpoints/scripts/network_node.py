#!/usr/bin/env python

import rospy
import json
import vlc
import os
import numpy as np
import network_client as server
import base64
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int8, String, Bool
from geometry_msgs.msg import PoseStamped
from navigation_msgs.msg import GoalWaypoint, VehicleState, EmergencyStop, LatLongPoint
from sensor_msgs.msg import NavSatFix, Image
from rospy.numpy_msg import numpy_msg

"""
This file contains the ros node: network node needed to be able 
to send the correct messages to the server. It talks to
network_client.
"""
class Network_Node(object):
        
    def __init__(self):
        rospy.init_node('network_node')
        
        stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        req_pub = rospy.Publisher('/destination_request', String, queue_size=10)
        location_speech_pub = rospy.Publisher('/location_speech', Bool, queue_size=10)
        gps_request_pub = rospy.Publisher('/gps_request', LatLongPoint, queue_size=10)
        safety_constant_pub = rospy.Publisher('/safety_constant', Bool, queue_size=10)
        safety_exit_pub = rospy.Publisher('/safety_exit', Bool, queue_size=10)

        rospy.Subscriber('/zed/image_raw', Image, passenger_image)
        rospy.Subscriber('/front_facing/image_raw', Image, front_camera_img)
        rospy.Subscriber('/current_position', Int8, sendPositionIndex)
        rospy.Subscriber('/vehicle_state', VehicleState, status_update)
        rospy.Subscriber('/pullover', Bool, pullover_callback)
        rospy.Subscriber('/speech_text', String, send_audio)
        rospy.Subscriber('/gps_coordinates', NavSatFix, sendLocation)
        rospy.Subscriber('/passenger_safe', Bool, passenger_safe_callback)
        rospy.Subscriber('/passenger_exit', Bool, passenger_exit_callback)
        

        self.empty = True

        
        self.exit_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "exit.mp3"))
        self.enter_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "enter.mp3"))
        rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            rate.sleep()
        
        
        
    ######################
    ### Sending Events ###
    ######################

    def arrived_dest(self):
        safety_exit_pub.publish(True)
        self.exit_sound.stop()
        self.exit_sound.play()
        server.send('arrived','/cart')
        
    def arrived_empty_dest(self):
        self.enter_sound.stop()
        self.enter_sound.play()
        server.send('arrived','/cart')

    def send_audio(self,msg):
        data = {
            "msg": msg.data,
            "id":id
        }
        server.send('audio',json.dumps(data))
        
        
    def send_location(self, msg):
        data = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'id': id
        }
        server.send('current_location',json.dumps(data))
        
    def send_passenger_exit(self):
        server.send('passenger_exit',id)

    def send_position_index(self, data):
        server.send("position", data.data)

    #pose tracking send when ready, AI start driving
    def send_ready(self):
        server.send('passenger_ready',id)

    #pose tracking send when unsafe
    def send_unsafe(self):
        selver.send('passenger_unsafe',id)

    #######################
    ### Other Functions ###
    #######################
    def location_finder(self, location_string):
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
        


    # sender_id is important to ensure all parties 
    # are ready to resume before releasing the stop command
    # ie both voice and pose tell us we need to stop and then
    # pose gives us the all clear but we should still be 
    # waiting for voice to also give the all clear
    # sender_id = 1 is the server, 2 is voice, 3 is pose, 4 is health monitor, 
    # 0 is for internal usage but is currently unused
    def pullover_callback(self, msg):
        if msg.data:
            stop_msg = EmergencyStop()
            stop_msg.emergency_stop = True
            stop_msg.sender_id = 2
            stop_pub.publish(stop_msg)
            self.send_unsafe()
        else:
            self.send_ready()

    def passenger_safe_callback(self, msg):
        if msg.data:
            self.send_ready()
        else:
            stop_msg = EmergencyStop()
            stop_msg.emergency_stop = True
            stop_msg.sender_id = 2
            stop_pub.publish(stop_msg)
            self.send_unsafe()

    def passenger_exit_callback(self, msg):
        self.send_passenger_exit()
        

    #Handles destination arrival as well as various other vehicle state changes
    def status_update(self, data):
        if not data.is_navigating:
            if data.reached_destination:
                if self.empty:
                    self.empty = False
                    self.arrived_empty_dest()
                else:
                    self.empty = True
                    self.arrived_dest()
                    
    #Processes and sends the image from the zed camera
    def passenger_image(self,msg):
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
        self.send('passenger_video', base64.b64encode(buffer))

    #Processes and sends the image from the front facing camera
    def front_camera_img(self, msg):
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
        self.send('front_facing_video', base64.b64encode(buffer))

if __name__ == "__main__":
    Network_Node()
        