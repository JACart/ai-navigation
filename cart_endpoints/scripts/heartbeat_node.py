#!/usr/bin/env python

import serial
import rospy
from navigation_msgs.msg import EmergencyStop
from std_msgs.msg import Int8, Bool
import time
cart_port = '/dev/ttyUSB0' #hardcoded depending on computer   SHOULD BE CHANGED BACK TO USB9 BEFORE COMMIT



class Heartbeat(object):

    def __init__(self):
        global cart_port
        self.node_rate = 10
        self.message_bytes = b''
        rospy.init_node('heartbeat_node')
        rospy.loginfo("Starting heartbeat node!")
        #Connect to arduino to receive messages
        try:
            self.heart_ser = serial.Serial(cart_port, 57600, timeout=2)
        except Exception as e:
            print( "heartbeat_node: " + str(e))
            rospy.logerr("heartbeat_node: " + str(e))

        rospy.loginfo("Heartbeat serial established")
    
        rate = rospy.Rate(self.node_rate)

        while not rospy.is_shutdown():
            self.message_bytes = self.heart_ser.read_until()
            print("Heartbeat message:")
            print("" + self.message_bytes)
            


if __name__ == "__main__":
    try:
        Heartbeat()
    except rospy.ROSInterruptException:
        pass
