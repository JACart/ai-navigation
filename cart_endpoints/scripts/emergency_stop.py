#!/usr/bin/env python

import socket
import rospy
from navigation_msgs.msg import emergency_stop

class EmergencyNode(object):

    def __init__(self):
        self.stop = False
        rospy.init_node('emergency_node')
        rospy.loginfo("Starting emergency node!")
        self.pub = rospy.Publisher('/emergency_stop', emergency_stop, queue_size=10)
        rospy.sleep(2)
        soc = socket.socket() 	  		 # Create a socket object
        host = socket.gethostname()                    # Get local machine name
        port = 12345 			 # Reserve a port for your service.
        soc.bind((host, port)) 			 # Bind to the port
        soc.listen(5) 			 # Now wait for client connection.
        while not rospy.is_shutdown():
            c, addr = soc.accept() 		# Establish connection with client.
            c.send('Emergency stop started'.encode())
            c.close() 			# Close the connection.
            self.stop = True
            while not rospy.is_shutdown():
                stop_message = emergency_stop()
                stop_message.header.stamp = rospy.Time.now()
                stop_message.emergency_stop = self.stop
                self.pub.publish(stop_message)

if __name__ == "__main__":
    try:
        EmergencyNode()
    except rospy.ROSInterruptException:
        pass

