#!/usr/bin/env python

import rospy
import math
import Control, PointToGoal, WaypointHandler
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from navigation_msgs.msg import EmergencyStop, LatLongPoint, WaypointsArray, VelAngle 

tolerance = 1.0
acceleration = 1.5 #acceleration in meters per second used by linear control function
hertz = 10 #rospy execution cycles/s
class TheOvermind(object):
    
    def __init__(self):
	global hertz
	global tolerance
	self.cmd_acceleration = 0.0 #used to tell the controller how fast to accelerate
	self.tolerance = tolerance #allowed imprecision for reaching a waypoint
	self.current_goal = None #current goal from list of goals
        self.vel_curr = 0.0 #current velocity
	self.kill = False
	#util classes
	self.waypoints = WaypointHandler.WaypointHandler(0)
	self.controller = Control.Control(hertz) 
	#publishers                                    
        self.vel_angle_p = rospy.Publisher('/vel_angle', VelAngle, queue_size=10)
        #subscribers	
	self.odom_s = rospy.Subscriber('/pose_and_speed', Odometry, self.odom_callback, queue_size=10)
	self.point_cloud_s = rospy.Subscriber('/2d_point_cloud', PointCloud2, self.point_cloud_callback, queue_size=10) 
	self.killswitch_s = rospy.Subscriber('/stop', EmergencyStop, self.killswitch_callback, queue_size=10) 
	self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback, queue_size=10) 
        rospy.init_node('TheOvermind', anonymous=False)
	self.control() 

    ''' this is the main event loop '''
    def control(self):
	global hertz
        rate = rospy.Rate(hertz) # 10hz
        while not rospy.is_shutdown():
       	    #if killswitch, publish 0 to vel_angle's vel_curr, and leave the angle the same
            
	    rate.sleep()
    
    def odom_callback(self, msg):
	self.odom_info = msg
	#if the final goal has not been reached, get the next goal
	if not self.waypoints.update_pos(self.odom_info, 0):
	    self.current_goal = get_goal()
	else:
	    self.current_goal = None
		
    def killswitch_callback(self, msg):
	if msg.emergency_stop:
	    self.kill = True

    def waypoints_callback(self, msg):
	self.waypoints.set_points(msg.waypoints)
	self.current_goal = get_goal()

    def point_cloud_callback(self, msg):
	pass
    def move_toward_goal():
	pass
    def decelerate_if_in_range():
	#if(dist == (math.exp(speed, 2)/2*acceleration):
	 #   self.control.set_acceleration
if __name__ == '__main__':
    try:
       TheOvermind()
    except rospy.ROSInterruptException:
        pass
