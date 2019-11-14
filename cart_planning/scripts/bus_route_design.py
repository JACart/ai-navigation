#!/usr/bin/env python 

import rospy
import os
import math
from geometry_msgs.msg import Point, PoseStamped
from navigation_msgs.msg import LocalPointsArray
from std_msgs.msg import String, Int8
from navigation_msgs.msg import GoalWaypoint

class bus_route_design():
    def __init__(self):
        rospy.init_node('bus_route_design')
        self.current_waypoint = 0
        self.waypoints_pub = rospy.Publisher('/global_path', LocalPointsArray, queue_size=10)
        self.current_pos_pub = rospy.Publisher('/current_position', Int8, queue_size=10)
        self.waypoints_sub = rospy.Subscriber('/local_points', LocalPointsArray, self.waypoints_callback, queue_size=10)
        self.path_request_sub = rospy.Subscriber('/path_request', GoalWaypoint, self.find_path_callback, queue_size=10)
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size = 10)
        rospy.spin()
    
    def waypoints_callback(self, msg):
        # Array of points that have the coordinates of each waypoints
        self.waypoint_list = msg.localpoints
        

    #Get the closest waypoint to the cart
    def pose_callback(self, msg):
        min_cost = 999999999
        pointarray = self.waypoint_list
        for i in range(len(pointarray)):
            cost = self.calculate_weight(pointarray[i], msg.pose.position)
            if cost < min_cost:
                self.current_waypoint = i
                min_cost = cost
                waypoint_msg = Int8()
                waypoint_msg.data = i
                self.current_pos_pub.publish(waypoint_msg)

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
        self.path_to_goal = [] # array of indexs
        path_found = False
        index = start

        while (not path_found):
            if index == goal: # reach the goal, then end.
                self.path_to_goal.append(self.waypoint_list[goal])
                path_found = True
                continue
            elif index == len(self.waypoint_list) - 1: # reach end of bus route
                self.path_to_goal.append(self.waypoint_list[index])
                index = 0 # go back to the beginning
                continue
            elif index == 20: # Intersection go left or right.
                self.path_to_goal.append(self.waypoint_list[index])
                if goal > 4 and goal < 20:
                    index = 5
                elif goal == 4:
                    index = 4                
                else:
				    index = 21
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

