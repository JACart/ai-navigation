#!/usr/bin/env python

import rospy
import math
import control, point_to_goal
import waypoint_hander.py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from navigation_msgs.msg import EmergencyStop, LatLongPoint, WaypointsArray 

hertz = 10 #rospy speed
class TheOvermind(object):
    
    def __init__(self):
        self.vel_curr = 0.0 #current velocity
	#publishers                                    
        self.cmd_vel_p = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #subscribers	
	self.odom_s = rospy.Subscriber('/pose_and_speed', Odometry, self.odom_callback, queue_size=10)
	self.point_cloud_s = rospy.Subscriber('/2d_point_cloud', PointCloud2, self.point_cloud_callback, queue_size=10) 
	self.killswitch_s = rospy.Subscriber('/stop', EmergencyStop, self.killswitch_callback, queue_size=10) 
	self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback, queue_size=10) 
        rospy.init_node('The Overmind', anonymous=False)
	self.control() 
 
    def control(self):
	global hertz
        rate = rospy.Rate(hertz) # 10hz
        while not rospy.is_shutdown():
       	    #if killswitch publish 0 to cmd_vel
            rate.sleep()    
    
    def odom_callback(self, msg):
        pass
    def killswitch_callback(self, msg):
	pass
    def waypoints_callback(self, msg):
        pass
    def point_cloud_callback(self, msg):
	pass
if __name__ == '__main__':
    try:
       TheOvermind()
    except rospy.ROSInterruptException:
        pass
