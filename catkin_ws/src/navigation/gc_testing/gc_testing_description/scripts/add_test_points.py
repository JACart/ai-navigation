#!/usr/bin/env python 

import rospy
from navigation_msgs.msg import WaypointsArray, LatLongPoint

class add_test_points(object):
    def __init__(self):
        rospy.init_node('add_test_points')

        self.waypoint_pub = rospy.Publisher('/waypoints', LatLongPoint, queue_size=10)

        f = open ("points.txt", "r")
        p_array = []
        
        for line in f:
            l = LatLongPoint()
            items = line.split(",")
            l.latitude = items[0]
            l.longitude = items[1]
            l.elevation = items[2]
            self.waypoint_pub.publish(l)
            #p_array.append(l)


        msg = WaypointsArray()
        msg.waypoints = p_array

        #self.waypoint_pub.publish(msg)
        print "done"

        rospy.spin()

if __name__ == "__main__":
    try:
	add_test_points()
    except rospy.ROSInterruptException:
	pass
