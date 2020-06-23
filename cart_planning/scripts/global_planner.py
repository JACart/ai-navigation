#!/usr/bin/env python 

import rospy
import os
import copy
import math
import gps_util
import simple_gps_util
import networkx as nx
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose, Point, PoseStamped, PointStamped
from navigation_msgs.msg import LocalPointsArray, LatLongPoint
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
        self.logic_graph = None
        
        # Get file pathing through ROS parameter server and load it
        file_name = rospy.get_param('graph_file')
        self.load_file(file_name)
        
        #Current waypoint of cart
        self.current_pos = None
        self.current_cart_node = None
        self.prev_cart_node = None

        #The points on the map
        self.local_array = None

        # Temporary solution for destinations, TODO: Make destinations embedded in graph nodes
        self.dest_dict = {"home":(22.9, 4.21), "cafeteria":(127, 140), "clinic":(63.3, 130), "reccenter":(73.7, 113), "office":(114, 117)}
        
        # self.location_req = rospy.Subscriber('/gps_request', String, self.request_callback, queue_size=10)

        # Listen for cart position changes
        self.pose_sub = rospy.Subscriber('/limited_pose', PoseStamped, self.pose_callback, queue_size=10)
        
        # Listen for standard destination requests
        self.dest_req_sub = rospy.Subscriber('/destination_request', String, self.request_callback, queue_size=10)
        
        # Clicked destination requests, accepted from RViz clicked points, disabled when building maps
        self.click_req_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        # This is here temporarily to test GPS_Util
        self.lat_long_req = rospy.Subscriber('/gps_request', LatLongPoint, self.gps_request_cb, queue_size=10)
        
        self.display_pub = rospy.Publisher('/display_gps', Marker, queue_size=10)
        
        # Publish the path to allow mind.py to begin navigation
        self.path_pub = rospy.Publisher('/global_path', LocalPointsArray, queue_size=10)

        # self.display_pub = rospy.Publisher('/path_display', Marker, queue_size=10)
        rospy.spin()
    
    # Load the graph file as the global graph
    def load_file(self, file_name):
        try:
            self.global_graph = nx.read_gml(file_name)
            for node in self.global_graph:
                self.global_graph.node[node]['active'] = True
            self.logic_graph = copy.deepcopy(self.global_graph)
        except:
            rospy.logerr("Unable to launch graph file pointed to in the constants file in .../cart_planning/launch")
    
    #If we receive a request for a destination
    def request_callback(self, msg):
        # ROS coordinates of the destination
        dest_pos = self.dest_dict[msg.data]

        # Prepare a point of the destination and send it off for calculation
        point = Point()
        point.x = dest_pos[0]
        point.y = dest_pos[1]

        self.calc_nav(point)

    def calc_nav(self, point):

        # Make a destination request when we don't know where the cart is at yet
        if self.current_pos is None:
            self.current_pos = PoseStamped()
            self.current_pos.pose.position.x = 0
            self.current_pos.pose.position.y = 0
        
        # Our current position in local coordinates and closest node to our position
        current_cart_pos = self.current_pos.pose.position
        rospy.loginfo("Before: " + str(self.current_cart_node))
        self.current_cart_node = self.get_closest_node(current_cart_pos.x, current_cart_pos.y, cart_trans=True)
        rospy.loginfo("After: " + str(self.current_cart_node))
        # Get the node closest to where we want to go
        destination_point = self.get_closest_node(point.x, point.y)

        # Remove our previous node to prevent searching directly behind cart
        
        name = None
        position = None
        status = None
        in_edges = None
        out_edges = None
        if self.prev_cart_node is not None:
            self.logic_graph = copy.deepcopy(self.global_graph)
            name = self.prev_cart_node
            position = self.global_graph.node[self.prev_cart_node]['pos']
            status = self.global_graph.node[self.prev_cart_node]['active']
            in_edges = copy.deepcopy(self.global_graph.in_edges(self.prev_cart_node, data=True))
            out_edges = copy.deepcopy(self.global_graph.out_edges(self.prev_cart_node, data=True))
            self.global_graph.remove_node(self.prev_cart_node)
        
        # Find shortest path from our current position to the destination node
        nodelist = nx.dijkstra_path(self.global_graph, self.current_cart_node, destination_point)
        
        self.logic_graph = copy.deepcopy(self.global_graph)
        # Re-add the previous node
        if name is not None:
            self.global_graph.add_node(name, pos=position, active=status)
            for u, v, data in in_edges:
                rospy.loginfo("Our new node")
                rospy.loginfo("In edge: u,v " + str(u) + "," + str(v))
                self.global_graph.add_edge(u, name, weight=data['weight'])
            
            for u, v, data in out_edges:
                rospy.loginfo("Out edge: u,v " + str(u) + "," + str(v))
                self.global_graph.add_edge(name, v, weight=data['weight'])

        
        # Set all nodes back to a state of not being a part of the previous/current path
        for node in self.global_graph:
            self.global_graph.node[node]['active'] = False

        # Convert our list of nodes to the destination to a list of points
        points_arr = LocalPointsArray()
        for node in nodelist:
            self.global_graph.node[node]['active'] = True
            new_point = Pose()
            new_point.position.x = self.global_graph.node[node]['pos'][0]
            new_point.position.y = self.global_graph.node[node]['pos'][1]
            points_arr.localpoints.append(new_point)
        
        self.logic_graph = copy.deepcopy(self.global_graph)
        self.path_pub.publish(points_arr)

    # Closest node to given point
    def get_closest_node(self, x, y, cart_trans=False):
        min_dist = 99999
        min_node = None
        
        # Just in case if the graph list in the for loop is outdated by a future update
        local_logic_graph = copy.deepcopy(self.logic_graph)

        for node in local_logic_graph.nodes:
            # Position of each node in the graph
            node_posx = local_logic_graph.node[node]['pos'][0]
            node_posy = local_logic_graph.node[node]['pos'][1]
            
            dist = self.dis(node_posx, node_posy, x, y)
            
            # Buffer, if we are trying to find the node closest to the cart, lets not count nodes the cart isnt actively following
            if cart_trans:
                if dist < min_dist and local_logic_graph.node[node]['active']:
                    min_node = node
                    min_dist = dist
            else:
                if dist < min_dist:
                    min_node = node
                    min_dist = dist
            
        # if we found a new closer node along our path let global planner know the current node and previous node
        if min_node is not self.current_cart_node and cart_trans:
                self.prev_cart_node = self.current_cart_node
                self.current_cart_node = min_node
                #rospy.loginfo("Current node: " + str(self.current_cart_node))
                #rospy.loginfo("Previous node: " + str(self.prev_cart_node))

        return min_node
    
    def dis(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    #Get the closest waypoint to the cart
    def pose_callback(self, msg):
        self.current_pos = msg
        self.current_cart_node = self.get_closest_node(msg.pose.position.x, msg.pose.position.y, cart_trans=True)

    # We've received a clicked point from RViz, calculate a path to it
    def point_callback(self, msg):
        self.calc_nav(msg.point)
    
    def cur_cart_pos(self):
        return self.get_closest_node(msg.pose.position.x, msg.pose.position.y)

    def gps_request_cb(self, msg):
        local_point = Point()
        y, x = simple_gps_util.latlon2xy(msg.latitude, msg.longitude, 38.433795, -78.862290)
        x = -(x)
        
        local_point.x = x
        local_point.y = y
        self.calc_nav(local_point)

        #print("X: " + str(x))
        #print("Y: " + str(y))
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
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        
        self.display_pub.publish(marker)
        
    def display_rviz(self, frame):
        local_display = copy.deepcopy(self.global_graph)
        i = 0
        for node in local_display.nodes:
            x = local_display.node[node]['pos'][0]
            y = local_display.node[node]['pos'][1]
            
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame

            marker.ns = "Path_NS"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = 0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration.from_sec(0.4)

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0

            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 0.6

            self.display_pub.publish(marker)
            i += 1
            
        for edge in local_display.edges:
            first_node = local_display.node[edge[0]]
            second_node = local_display.node[edge[1]]
            
            points = []
            
            first_point = Point()
            first_point.x = first_node['pos'][0]
            first_point.y = first_node['pos'][1]
            
            second_point = Point()
            second_point.x = second_node['pos'][0]
            second_point.y = second_node['pos'][1]
            
            points.append(first_point)
            points.append(second_point)
            
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame

            marker.ns = "Path_NS"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = 0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration.from_sec(0.4)

            marker.points = points

            marker.scale.x = 0.3
            marker.scale.y = 0.6
            marker.scale.z = 0

            self.display_pub.publish(marker)
            i += 1

if __name__ == "__main__":
    try:
	    global_planner()
    except rospy.ROSInterruptException:
	    pass