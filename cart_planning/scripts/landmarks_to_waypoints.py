#!/usr/bin/env python 

import rospy
import os
from navigation_msgs.msg import WaypointsArray, LatLongPoint, Landmarks
from sensor_msgs.msg import NavSatFix

class landmarks_to_waypoints(object):
    def __init__(self):
        rospy.init_node('landmarks_to_waypoints')

        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointsArray, queue_size=10, latch=True)
        self.landmark_sub = rospy.Subscriber('/landmarks', Landmarks, self.landmark_callback, queue_size=10)
        
        rospy.spin()

    def landmark_callback(self, msg):
        landmarks_waypoints = msg.landmarks       
        f = open (os.path.dirname(__file__)+"/"+landmarks_waypoints.data+".txt", "r")
        p_array = []
        
        for line in f:
            l = NavSatFix()
            items = line.split(",")
            l.latitude =  (float) (items[0])
            l.longitude = (float) (items[1])
            l.altitude = (float) (items[2].split("\n")[0])

            p_array.append(l)


        msg = WaypointsArray()
        msg.waypoints = p_array
        self.waypoint_pub.publish(msg)
        print "Waypoints added for: " + msg.landmarks.data 

if __name__ == "__main__":
    try:
	    landmarks_to_waypoints()
    except rospy.ROSInterruptException:
	    pass
