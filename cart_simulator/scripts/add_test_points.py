#!/usr/bin/env python 

import rospy
import os
from navigation_msgs.msg import WaypointsArray, LatLongPoint
from sensor_msgs.msg import NavSatFix

class add_test_points(object):
    def __init__(self):
        rospy.init_node('add_test_points')

        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointsArray, queue_size=10, latch=True)

        f = open (os.path.dirname(__file__)+"/points.txt", "r")
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
        #for i in range(100):

        self.waypoint_pub.publish(msg)
            #rospy.sleep(.1)
        #print msg
        print "done"

        rospy.spin()

if __name__ == "__main__":
    try:
	    add_test_points()
    except rospy.ROSInterruptException:
	    pass
