#!/usr/bin/env python

import rospy
import math
import controller, point_to_goal, waypoint_handler
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from navigation_msgs.msg import EmergencyStop, LatLongPoint, WaypointsArray, VelAngle 
max_speed = 2.2352 #5 mph converted to m/s (temporary max)
tolerance = 1.0
acceleration = 1.5 #acceleration in meters per second used by controller
hertz = 10 #rospy execution cycles/s
class TheOvermind(object):
    
    def __init__(self):
	global hertz
	global tolerance
	self.odom = None
	self.tolerance = tolerance #allowed imprecision for reaching a waypoint
	self.current_goal = None #current goal from list of goals
        self.vel_curr = 0.0 #current velocity
	self.kill = False
	self.vel_angle = VelAngle()
	#util classes
	self.waypoints = waypoint_handler.WaypointHandler(self.tolerance)
	self.controller = controller.Controller(hertz)
	print self.controller.millis_since_epoch()
	#publishers                                    
        self.vel_angle_p = rospy.Publisher('/vel_angle', VelAngle, queue_size=10)
        #subscribers	
	self.odom_s = rospy.Subscriber('/pose_and_speed', Odometry, self.odom_callback, queue_size=10)
	self.point_cloud_s = rospy.Subscriber('/2d_point_cloud', PointCloud2, self.point_cloud_callback, queue_size=10) 
	self.killswitch_s = rospy.Subscriber('/stop', EmergencyStop, self.killswitch_callback, queue_size=10) 
	self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback, queue_size=10) 

        rospy.init_node('the_overmind', anonymous=False)
	self.control() 

    ''' main event loop '''
    def control(self):
	global hertz
        rate = rospy.Rate(hertz) # 10hz
        while not rospy.is_shutdown():
       	    #if killswitch, publish 0 to vel_angle's vel_curr, and leave the angle the same	
	    self.controller_handler()
	    self.vel_angle_p.publish(self.vel_angle)
	    rate.sleep()
    
    def odom_callback(self, msg):
	self.odom = msg
	self.vel_angle.vel_curr = vector3_to_scalar(msg.twist.twist.linear)
	#if the final goal has not been reached, get the next goal
	if not self.waypoints.update_pos(msg, 0):
	    self.current_goal = get_goal()
	else:
	    self.current_goal = None

    ''' set a flag to true if emergency stop is activated '''		
    def killswitch_callback(self, msg):
	if msg.emergency_stop:
	    self.kill = True

    def waypoints_callback(self, msg):
	#replace the waypoints array with new data
	self.waypoints.set_points(msg.waypoints)
	self.current_goal = get_goal()

    ''' handles controller related things in the main loop '''
    def controller_handler(self):
	global acceleration
	global max_speed
	self.controller.step()
	if self.current_goal and self.actual_speed == 0.0:
	    self.controller.accelerate(acceleration, max_speed)
	elif self.in_deceleration_range():
	    self.controller.accelerate(acceleration, 0) 
	self.vel_angle.vel = self.controller.get_velocity()
	self.vel_angle.angle = self.controller.get_angle()
   
    def is_approx(actual, expected, tolerance):
	return expected - tolerance < actual < expected + tolerance
 
    def point_cloud_callback(self, msg):
	pass

    ''' Checks if in range to start decelerating (the range is based on tolerance). 
        The car is in range when, if it were to decelerate, the time it takes to reach 0 distance from the goal 
	is about equivalent to the time it takes to reach a speed of zero.
	the equation dist = speed^2/2*acceleration indicates this. A constant
	acceleration is used to find this equivalence '''
    def in_deceleration_range(self):
	global acceleration #hardcoded acceleration constant
	if not (self.waypoints and self.waypoints.waypoints):
	    return False
	dist = waypoints.distance_from_goal()
	return is_approx(dist, (self.speed * self.speed)/2*acceleration, self.tolerance)
  
if __name__ == '__main__':
    try:
       TheOvermind()
    except rospy.ROSInterruptException:
       pass
