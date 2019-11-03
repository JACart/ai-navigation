#!/usr/bin/env python 

import rospy
import os
import math
import networkx as nx
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, PoseStamped
from navigation_msgs.msg import LocalPointsArray
from std_msgs.msg import String

class global_planner(object):
    def __init__(self):
        rospy.init_node('global_planner')

        #Create a new directional graph
        self.global_graph = nx.DiGraph()

        #Current waypoint of cart
        self.current_waypoint = 0

        #The points on the map
        self.local_array = None
        self.local_sub = rospy.Subscriber('/local_points', LocalPointsArray,
                                            self.localpoints_callback, queue_size=10)
        self.location_req = rospy.Subscriber('/location_requests', String, self.request_callback, queue_size=10)

        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size=10)

        self.path_pub = rospy.Publisher('/global_path', LocalPointsArray, queue_size=10)
        rospy.spin()

    #Build the graph once all points are loaded
    def localpoints_callback(self, msg):
        # self.local_array = msg.localpoints
        pointarray = self.local_array = msg.localpoints
        graph = self.global_graph
        for i in range(len(pointarray)):
            graph.add_node(i)
            if i > 0:
                cost = self.calculate_weight(pointarray[i], pointarray[i-1])
                graph.add_edge(i - 1, i, weight=cost)
            self.hardcode_checks(i)
     
    #If we receive a request for a destination
    def request_callback(self, msg):
        location = msg.data
        pathlist = None
        finalpath = []
        pubmessage = LocalPointsArray()
    
        if location == 'Cafe':
            rospy.loginfo('Navigation to cafe')
            print(str(self.current_waypoint))
            pathlist = nx.dijkstra_path(self.global_graph, self.current_waypoint, 4)
            for p in pathlist:
                finalpath.append(self.local_array[p])
            pubmessage.localpoints = finalpath
            rospy.loginfo('Publishing new pathing')
            print(finalpath)
            self.path_pub.publish(pubmessage)
        else:
            pathlist = self.local_array
            pubmessage.localpoints = pathlist
            self.path_pub.publish(pubmessage)
    
    #Calculate the cost from one point to the next (distance)
    def calculate_weight(self, point1, point2):
        dX = (point2.x - point1.x)**2
        dY = (point2.y - point1.y)**2

        return math.sqrt(dX + dY)
    
    #Specifically for linking left/right turn node with the node of the beginning of the loop
    def hardcode_checks(self, i):
        if i == 20:
            cost = self.calculate_weight(self.local_array[i], self.local_array[i-1])
            self.global_graph.add_edge(i, 4, weight=cost)

    #Get the closest waypoint to the cart
    def pose_callback(self, msg):
        pointarray = self.local_array
        for i in range(len(pointarray)):
            cost = self.calculate_weight(pointarray[i], msg.pose.position)
            if cost < 1:
                self.current_waypoint = i


if __name__ == "__main__":
    try:
	    global_planner()
    except rospy.ROSInterruptException:
	    pass
