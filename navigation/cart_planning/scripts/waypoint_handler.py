#!/usr/bin/env python

import rospy
import gps_util
import geometry_util


from geometry_msgs.msg import Point

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

    ''' called when we are updated with a new list of points (the goal has been changed).
    The old waypoints are forgotten/ignored completely. '''
    def set_points(self, array):
        self.waypoints = gps_array_to_xyz_array(array)
        self.goal_index = 0
        self.curr_pos = None

    ''' increments the current point index and returns false if there are no more indices.
    should be called when the previous goal has been reached '''
    def next_point(self):
        if self.goal_index >= len(self.waypoints)-1:
            return False
        else:
            self.goal_index += 1
            return True

    ''' rough estimate of distance from goal (assumes perfect turning)
    adds up the distance between the current position and the first waypoint
    and between each waypoint not yet reached'''
    def distance_from_end(self):
        distance = 0
        if not self.waypoints:
            return distance
        distance += self.distance_from_next()
        for i in range(self.goal_index, len(self.waypoints)-1):
            point = self.waypoints[i]
            point2 = self.waypoints[i+1]
            distance += geometry_util.xyz_dist_between_points(point, point2)
        return distance

    ''' indicates whether the next point has been reached '''
    def reached_next_point(self):
        if not self.waypoints:
            return False
        return abs(self.distance_from_next()) < self.tolerance
    ''' angle between car and the next point in degrees '''
    def angle_from_next(self):
        goal = self.get_goal()
        if not goal:
            return None
        return geometry_util.xy_angle_between_points(self.curr_pos.pose.pose.position, goal)

    ''' distance between car and the next point '''
    def distance_from_next(self):
        goal = self.get_goal()
        if not goal:
            return None
        return geometry_util.xyz_dist_between_points(self.curr_pos.pose.pose.position, goal)

    ''' current position is a message of type nav_msgs/Odometry.
    Returns true if the final goal point has been reached'''
    def update_pos(self, current_position, tolerance):
        if not current_position:
            return False
        self.curr_pos = current_position
        self.tolerance = self.min_tolerance if (self.min_tolerance > tolerance) else tolerance
        return self.reached_next_point() and not self.next_point()

    ''' retrieves the current goal '''
    def get_goal(self):
        point = self.get_point_at_index(self.goal_index)
        if not point:
            return None
        return point

    def get_point_at_index(self, index):
        if not self.waypoints or index >= len(self.waypoints):
            return None
        goal = self.waypoints[index]
        xyz_goal = gps_util.get_point(goal.latitude, goal.longitude)
        goal_point = Point()
        goal_point.x = xyz_goal.x
        goal_point.y = xyz_goal.y
        goal_point.z = 0 #elevation is always 0
        return goal_point

'''
Takes an array of gps coordinates and returns an
array with the corresponding xyz coordinates
'''
def gps_array_to_xyz_array(gps_array):
    return geometry_util.add_intermediate_points(
        [gps_util.get_point(gps.latitude, gps.longitude)
         for gps in gps_array], 1)
if __name__ == '__main__':
    try:
        WaypointHandler(5.0)
    except rospy.ROSInterruptException:
        pass
