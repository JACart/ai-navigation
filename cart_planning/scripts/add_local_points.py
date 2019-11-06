#!/usr/bin/env python 

import rospy
import os
from geometry_msgs.msg import Point
from navigation_msgs.msg import LocalPointsArray

class add_local_points(object):
    def __init__(self):
        rospy.init_node('add_local_points')

        self.waypoint_pub = rospy.Publisher('/local_points', LocalPointsArray, queue_size=10, latch=True)

        f = open (os.path.dirname(__file__)+"/buspoints.txt", "r")
        p_array = []
        
        for line in f:
            new_point = Point()
            items = line.split(",")
            new_point.x = (float) (items[0])
            new_point.y =  (float) (items[1])

            p_array.append(new_point)


        msg = LocalPointsArray()
        msg.localpoints = p_array
        #for i in range(100):

        self.waypoint_pub.publish(msg)
            #rospy.sleep(.1)
        #print msg
        print "done"

        rospy.spin()

if __name__ == "__main__":
    try:
	    add_local_points()
    except rospy.ROSInterruptException:
	    pass
