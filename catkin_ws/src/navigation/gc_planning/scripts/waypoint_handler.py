#!/usr/bin/env python

import rospy
import math
import point_to_goal
from navigation_msgs.msg import WaypointsArray, LatLongPoint 

class WaypointHandler(object):
    ''' must supply a minimum tolerance -- the distance threshold between
	current location and target where the cart is considered to have
	reached the target waypoint. '''
    def __init__(self, min_tolerance):
	self.goal_index = None #index of the current target in waypoints array
	self.waypoints = None #waypoints are of type LatLongPoint
	self.curr_pos = None 
	self.min_tolerance = min_tolerance
	self.tolerance = min_tolerance

    ''' current actual location updated from gps (pose_and_speed topic) ''' 
    def set_curr_location(self, odom_data, tolerance):
	#the tolerance passed into this method must somehow be calculated from the Odometry
	#message's pose covariance matrix
	self.curr_pos = odom_data
	self.tolerance = self.min_tolerance if (self.min_tolerance > tolerance) else tolerance
	    #tolerance is updated based on certainty of current location, minimum tolerance is
	    #used if tolerance is too low

    ''' called when we are updated with a new list of points (the goal has been changed).
	The old waypoints are forgotten/ignored completely. '''	
    def set_points(self, array):
	self.waypoints = array
	self.goal_index = 0

    ''' increments the current point index and returns true if there are no more indices.
	should be called when the previous goal has been reached '''
    def next_point(self):
	if(self.goal_index >= self.waypoints.length()-1):
	    return True
	else:
	    goal_index += 1
	    return False

    ''' rough estimate of distance from goal (assumes perfect turning) 
	adds up the distance between the current position and the first waypoint
	and between each waypoint not yet reached'''
    def distance_from_goal(self):
	distance = 0
	if not self.waypoints:
	    return distance
	distance += self.distance_from_next() 
	for i in range(self.goal_index, waypoints.length()-1):
 	    point = self.waypoints[i]
	    point2 = self.waypoints[i+1]
	    distance += point_to_goal.distance_between_points(
		point.latitude, point.longitude, point2.latitude, point2.longitude
			    )
	return distance
				    
    ''' indicates whether the next point has been reached ''' 
    def reached_next_point(self):
	if not self.waypoints:
	    return False
	goal = self.waypoints[self.goal_index]
	return math.abs(self.distance_from_next()) < self.tolerance

    ''' distance between car and the next point ''' 
    def distance_from_next(self):
	if not self.waypoints:
	    return 
	return point_to_goal.distance_between_points(
			self.waypoints[self.goal_index].latitude,
			self.waypoints[self.goal_index].longitude,
			self.curr_pos.pose.pose.position.x,
			self.curr_pos.pose.pose.position.y) #not sure if x and y agree with lat and long
   
    ''' current position is a message of type nav_msgs/Odometry. 
	Returns true if the final goal point has been reached'''
    def update_pos(self, current_position, tolerance):
	self.set_curr_location(current_position, tolerance)
	return self.reached_next_point() and self.next_point()
	#this line will call the necessary methods -- only return true when the final goal has been reached	
    
    ''' retrieves the current goal '''
    def get_goal(self):
	if not self.waypoints:
	    return None
	return self.waypoints[self.goal_index]	

if __name__ == '__main__':
    try:
       WaypointHandler()
    except rospy.ROSInterruptException:
        pass
