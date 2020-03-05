#!/usr/bin/env python 

import rospy
import os
import math
import gps_util
import simple_gps_util
import networkx as nx
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped
from navigation_msgs.msg import LocalPointsArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Header

class global_planner(object):
    def __init__(self):
        rospy.init_node('global_planner')

        self.anchor_lat = 38.432150 
        self.anchor_long = -78.876106
        
        #Create a new directional graph
        self.global_graph = nx.DiGraph()
        
        # TODO Set file pathing through ROS param server
        self.load_file("/home/jeffercize/catkin_ws/src/ai-navigation/XLabsGraphV1.gml")
        
        #Current waypoint of cart
        self.current_pos = None

        #The points on the map
        self.local_array = None
        
        # self.location_req = rospy.Subscriber('/gps_request', String, self.request_callback, queue_size=10)

        # Listen for cart position changes
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size=10)
        
        # Listen for standard destination requests
        self.dest_req_sub = rospy.Subscriber('/destination_request', Point, self.request_callback, queue_size=10)
        
        # Clicked destination requests, accepted from RViz clicked points, disabled when building maps
        # self.click_req_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        # This is here temporarily to test GPS_Util
        self.lat_long_req = rospy.Subscriber('/gps_request_test', NavSatFix, self.gps_request_cb, queue_size=10)
        
        self.display_pub = rospy.Publisher('/display_gps', Marker, queue_size=10)
        
        # Publish the path to the destination
        self.path_pub = rospy.Publisher('/global_path', LocalPointsArray, queue_size=10)
        rospy.spin()
    
    # Load the graph file as the global graph
    def load_file(self, file_name):
        self.global_graph = nx.read_gml(file_name)
    
    #If we receive a request for a destination
    def request_callback(self, msg):
        
        # Make a destination request when we don't know where the cart is at yet
        if self.current_pos is None:
            self.current_pos = PoseStamped()
            self.current_pos.pose.position.x = 0
            self.current_pos.pose.position.y = 0
        
        # Our current position in local coordinates and its closest node
        current_cart_pos = self.current_pos.pose.position
        current_cart_node = self.get_closest_node(current_cart_pos.x, current_cart_pos.y)
        
        # Get the node closest to where we want to go
        destination_point = self.get_closest_node(msg.x, msg.y)
        
        # Find shortest path from our current position to the destination node
        nodelist = nx.dijkstra_path(self.global_graph, current_cart_node, destination_point)
        
        # Convert our list of nodes to the destination to a list of points
        points_arr = LocalPointsArray()
        for node in nodelist:
            new_point = Pose()
            new_point.position.x = self.global_graph.node[node]['pos'][0]
            new_point.position.y = self.global_graph.node[node]['pos'][1]
            points_arr.localpoints.append(new_point)
        
        self.path_pub.publish(points_arr)

    # Closest node to given point
    def get_closest_node(self, x, y):
        min_dist = 99999
        min_node = None
        
        for node in self.global_graph.nodes:
            # Position of each node in the graph
            node_posx = self.global_graph.node[node]['pos'][0]
            node_posy = self.global_graph.node[node]['pos'][1]
            
            dist = self.dis(node_posx, node_posy, x, y)
            
            if dist < min_dist:
                min_node = node
                min_dist = dist
                
        return min_node
    
    def dis(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    #Get the closest waypoint to the cart
    def pose_callback(self, msg):
        self.current_pos = msg
        
    # We've received a clicked point from RViz, process it as a destination request
    def point_callback(self, msg):
        self.request_callback(msg.point)
    
    def cur_cart_pos(self):
        return self.get_closest_node(msg.pose.position.x, msg.pose.position.y)

    def gps_request_cb(self, msg):
        # local_point = gps_util.get_point(msg)
        local_point = Point()
        point_angle = gps_util.direction_between_coordinates(self.anchor_lat, self.anchor_long, 38.432264, -78.876032)
        print(point_angle)
        local_point = gps_util.get_point(msg)
        # x, y = simple_gps_util.latlon2xy(msg.latitude, msg.longitude, self.anchor_lat, self.anchor_long)
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "/map"
        
        marker.ns = "GPS_NS"
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = 0
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration.from_sec(10)
        
        marker.pose.position.x = local_point.x
        marker.pose.position.y = local_point.y
        marker.pose.position.z = 0.0
        
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        
        self.display_pub.publish(marker)
        

if __name__ == "__main__":
    try:
	    global_planner()
    except rospy.ROSInterruptException:
	    pass
