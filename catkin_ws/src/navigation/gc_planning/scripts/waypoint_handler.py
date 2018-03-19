#!/usr/bin/env python

import rospy
import math
import point_to_goal
from navigation_msgs.msg import WaypointsArray, LatLongPoint 

class WaypointHandler(object):
    ''' must supply a minimum tolerance -- the distance threshold between
	current location and target wherein the cart is considered to have
	reached the target waypoint. '''
    def __init__(self, min_tolerance):
	self.goal_index = None
	self.waypoints = None
	self.curr_pos = None
	self.min_tolerance = min_tolerance
	self.tolerance = min_tolerance
    ''' current actual location updated from gps (pose_and_speed topic) ''' 
    def set_curr_location(self, odom_point, tolerance):
	#the tolerance passed into this method must somehow be calculated from the Odometry
	#message's pose covariance matrix
	self.curr_pos = odom_point
	self.tolerance = min_tolerance if (min_tolerance > tolerance) else tolerance
	    #tolerance is updated based on certainty of current location, minimum tolerance is
	    #used if tolerance is too low
    ''' called when we are updated with a new list of points (the goal has been changed).
	The old waypoints are forgotten/ignored completely. '''	
    def set_points(self, array):
	self.waypoints = array
	self.goal_index = 0
    ''' increments the current point index and returns true if the final goal index has been reached.
	should be called when the previous goal has been reached '''
    def next_point(self):
	if(self.goal_index >= self.waypoints.length()-1)
	    return true
	else
	    goal_index += 1
	    return false
	    
    ''' indicates whether the next point has been reached ''' 
    def reached_next_point(self):
	goal = self.waypoints[self.goal_index]
	return (math.abs
		(point_to_goal.distance_between_points
			(goal.latitude, goal.longitude, 
			 self.curr_loc.pose.pose.x, self.curr_loc.pose.pose.y)) 
		< tolerance)
    ''' current position is a message of type nav_msgs/Odometry. 
	Returns true if the final goal point has been reached'''
    def update_pos(self, current_position, tolerance):
	self.set_curr_location(current_position, tolerance)
	return self.reached_next_point() && self.next_point()
	#this line will call the necessary methods -- only returns
	#true when the final goal has been reached	
    def get_goal(self):
	return self.waypoints[self.goal_index]	
if __name__ == '__main__':
    try:
       WaypointHandler()
    except rospy.ROSInterruptException:
        pass
