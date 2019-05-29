#!/usr/bin/env python

import rospy
import math
import gps_util
from navigation_msgs.msg import WaypointsArray
from sensor_msgs.msg import NavSatFix 

class WaypointHandler(object):
    ''' must supply a minimum tolerance -- the distance threshold between
    current location and target where the cart is considered to have
    reached the target waypoint. '''
    def __init__(self, min_tolerance):
        self.goal_index = None #index of the current target in waypoints array
        self.waypoints = None #waypoints are of type NavSatFix
        self.curr_pos = None
        self.min_tolerance = min_tolerance
        self.tolerance = min_tolerance

    def gps_array_to_xyz_array(self, gps_array):
        return gps_util.add_intermediate_points(
            [
             gps_util.lat_long_to_xyz(gps.latitude, gps.longitude) for gps in gps_array
            ],
            1
        )
    ''' called when we are updated with a new list of points (the goal has been changed).
    The old waypoints are forgotten/ignored completely. '''
    def set_points(self, array):
        self.waypoints = self.gps_array_to_xyz_array(array);
        self.goal_index = 0
        self.curr_pos = None

    ''' increments the current point index and returns false if there are no more indices.
    should be called when the previous goal has been reached '''
    def next_point(self):
        if self.goal_index >= self.waypoints.length()-1:
            return False
        else:
            goal_index += 1
            return True

    ''' rough estimate of distance from goal (assumes perfect turning) 
    adds up the distance between the current position and the first waypoint
    and between each waypoint not yet reached'''
    def distance_from_end(self):
        distance = 0
        if not self.waypoints:
            return distance
        distance += self.distance_from_next()
        for i in range(self.goal_index, waypoints.length()-1):
            point = self.waypoints[i]
            point2 = self.waypoints[i+1]
            distance += gps_util.xyz_distance_between_points(point, point2)
        return distance

    ''' indicates whether the next point has been reached '''
    def reached_next_point(self):
        if not self.waypoints:
            return False
        return math.abs(self.distance_from_next()) < self.tolerance
    ''' angle between car and the next point in degrees '''
    def angle_from_next(self):
        goal = self.get_goal()
        if not goal:
            return None
        return gps_util.xy_angle_between_points(self.curr_pos.pose.pose.position, goal)

    ''' distance between car and the next point '''
    def distance_from_next(self):
        goal = self.get_goal()
        if not goal:
            return None
        return gps_util.xyz_dist_between_points(self.curr_pos.pose.pose.position, goal)

    ''' current position is a message of type nav_msgs/Odometry. 
    Returns true if the final goal point has been reached'''
    def update_pos(self, current_position, tolerance):
        if not current_position:
            return False
        self.curr_pos = current_position
        self.tolerance = self.min_tolerance if (self.min_tolerance > tolerance) else tolerance
        return self.reached_next_point() and not self.next_point()
        #this line will call the necessary methods -- only return true when the final goal has been reached

    ''' retrieves the current goal '''
    def get_goal(self):
        point = self.get_point_at_index(self.goal_index)
        if not point:
            return None
        return point
        
    def get_point_at_index(self, index):
        if not self.waypoints or index >= self.waypoints.length():
            return None
        goal = self.waypoints[index]
        xyz_goal = gps_util.lat_long_to_xyz(goal.latitude, goal.longitude)
        goal_point = Point()
        goal_point.x = xyz_goal[0] #x in tuple
        goal_point.y = xyz_goal[1] #y in tuple
        goal_point.z = 0 #elevation is always 0 for now
        return goal_point

if __name__ == '__main__':
    try:
       WaypointHandler()
    except rospy.ROSInterruptException:
        pass
