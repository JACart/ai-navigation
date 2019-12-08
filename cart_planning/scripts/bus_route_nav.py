#!/usr/bin/env python 

import rospy
import os
import math
from geometry_msgs.msg import Point, PoseStamped
from navigation_msgs.msg import LocalPointsArray, VehicleState, LatLongPoint
from std_msgs.msg import String, Int8
from navigation_msgs.msg import GoalWaypoint

class bus_route_design():
    def __init__(self):
        rospy.init_node('bus_route_nav')
        self.current_waypoint = 0
        self.waypoint_list = None
        self.gps_list = None
        self.waypoints_pub = rospy.Publisher('/global_path', LocalPointsArray, queue_size=10)
        self.current_pos_pub = rospy.Publisher('/current_position', Int8, queue_size=10)
        self.waypoints_sub = rospy.Subscriber('/local_points', LocalPointsArray, self.waypoints_callback, queue_size=10)
        self.gps_sub = rospy.Subscriber('/gps_path', LocalPointsArray, self.gps_callback, queue_size=10)
        self.path_request_sub = rospy.Subscriber('/path_request', GoalWaypoint, self.find_path_callback, queue_size=10)
        self.path_request_pub = rospy.Publisher('/path_request', GoalWaypoint, queue_size=10)
        self.gps_request_sub = rospy.Subscriber('/gps_request', LatLongPoint, self.find_gps_callback, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size = 10)
        self.vehicle_state_pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size=10)        
        rospy.spin()
    
    def gps_callback(self, msg):
        self.gps_list = msg.localpoints

    def waypoints_callback(self, msg):
        # Array of points that have the coordinates of each waypoints
        self.waypoint_list = msg.localpoints
        
    def turn_range(self, angle1, angle2):
        phi = abs(angle2-angle1)
        if phi > 180:
            result = 360-phi
        else:
            result = phi
        return result

    #Get the closest waypoint to the cart
    def pose_callback(self, msg):
        min_cost = 999999999
        pointarray = self.waypoint_list
        for i in range(len(pointarray)):
            cost = self.calculate_weight(pointarray[i].position, msg.pose.position)     
            if cost < min_cost and self.turn_range(pointarray[i].orientation.z, (msg.pose.orientation.z + 1) * 180) < 60:
                self.current_waypoint = i
                min_cost = cost
        waypoint_msg = Int8()
        waypoint_msg.data = self.current_waypoint
        self.current_pos_pub.publish(waypoint_msg)

    # Process a GPS location request
    def find_gps_callback(self, msg):
        min_ind = 0
        min_cost = 9999999
        lat_long_arr = self.gps_list
        for i in range(len(lat_long_arr)):
            rospy.loginfo("Latitude: " + str(lat_long_arr[min_ind].position.x) + " Longitude: " + str(lat_long_arr[i].position.y))
            cost = self.calculate_weight(lat_long_arr[min_ind].position, lat_long_arr[i].position)
            if cost < min_cost:
                min_ind = i
                min_cost = cost
        path_msg = GoalWaypoint()
        path_msg.start = -1
        path_msg.goal = min_ind

        self.path_request_pub.publish(path_msg)

    #Calculate the cost from one point to the next (distance)
    def calculate_weight(self, point1, point2):
        dX = (point2.x - point1.x)**2
        dY = (point2.y - point1.y)**2

        return math.sqrt(dX + dY)

    #Calculate the path from the start (current location) to the goal
    def find_path_callback(self, msg):
        if(msg.start == -1): 
            start = self.current_waypoint
        else:
            start = msg.start
        goal = msg.goal
        if(start == goal):
            current_state = VehicleState()
            current_state.is_navigating = False
            current_state.reached_destination = True
            self.vehicle_state_pub.publish(current_state)
            return
        self.path_to_goal = [] # array of indexs
        path_found = False
        index = start
        if(self.waypoint_list == None):
            return
        while (not path_found):
            if index == goal: # reach the goal, then end.
                self.path_to_goal.append(self.waypoint_list[goal])
                path_found = True
                continue
            elif index == len(self.waypoint_list) - 1: # reach end of bus route
                self.path_to_goal.append(self.waypoint_list[index])
                index = 0 # go back to the beginning
                continue
            elif index == 23: # Intersection go left or right.
                self.path_to_goal.append(self.waypoint_list[index])
                if goal > 5 and goal < 23:
                    index = 6
                elif goal == 5:
                    index = 5
                else:
				    index = 24
                continue
            self.path_to_goal.append(self.waypoint_list[index]) # just following bus route numerically
            if index == goal:
                path_found = True #Protects against edge case of goal index 5
            index += 1
        
        # Make and Publish msg to WaypointsArray
        msg = LocalPointsArray()
        msg.localpoints = self.path_to_goal
        self.waypoints_pub.publish(msg)
		
	
if __name__ == '__main__':
    try:
        bus_route_design()
    except rospy.ROSInterruptException:
        pass

