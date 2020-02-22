#!/usr/bin/env python 
import curses
import rospy
import sys
import time
import math
import copy
import networkx as nx
import datetime

from navigation_msgs.msg import VelAngle
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Bool, Header
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

class PathCreation(object):
    def __init__(self):
        rospy.init_node('path_creation_tool')
        self.param_change = rospy.Publisher('/realtime_param_change', Int8, queue_size=10)
        self.a_param_change = rospy.Publisher('/realtime_a_param_change', Int8, queue_size=10)
        self.debug_change = rospy.Publisher('/realtime_debug_change', Bool, queue_size=10)
        self.display_pub = rospy.Publisher('/path_display', Marker, queue_size=10)
        
        self.point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        self.point_mode = "Add" # Select, Add, Remove, Connect ...a point
        
        self.prev_key = 1
        
        self.global_graph = nx.DiGraph()
        self.last_node = None
        self.selected_node = None
        self.node_count = 0
        
        # When adding new nodes, auto-connect previous node to new node
        self.auto_connect = True
        
        # For node selection 
        self.first_selection = None
        self.second_selection = None
        
        self.display_graph = None
        
        curses.wrapper(self.get_input)
        
        
        
    def point_callback(self, msg):
        node_x = msg.point.x
        node_y = msg.point.y
        
        if self.point_mode is "Add":
            self.add_point(node_x, node_y)  
        elif self.point_mode is "Remove":
            self.remove_point(node_x, node_y)
        elif self.point_mode is "Connect":
            self.connect_point(node_x, node_y)
        
        #Prevent display function from handling outdated graph attributes
        self.display_graph = copy.deepcopy(self.global_graph)

    def add_point(self, x, y):
        node_name = 'Node:' + str(self.node_count)
        self.global_graph.add_node(self.node_count, name=node_name, pos=[x, y])
            
            
        #Connect previous node to current node
        if self.last_node >= 0 and self.auto_connect:
            self.add_weighted_edge(self.last_node, self.node_count)
        
        self.last_node = self.node_count
        self.node_count += 1
        
    def remove_point(self, x, y):
        # TODO checking for empty graph here
        # Find the closest node to where the user clicked
        min_node = get_closest_node(x, y)
        
        self.node_count -= 1
        self.last_node -= 1
        self.global_graph.remove_node(min_node)
            
    def add_weighted_edge(self, first_node, second_node):
        pos_tuple_last = self.global_graph.node[first_node]['pos']
        pos_tuple_curr = self.global_graph.node[second_node]['pos']
        
        x1 = pos_tuple_last[0]
        y1 = pos_tuple_last[1]
        x2 = pos_tuple_curr[0]
        y2 = pos_tuple_curr[1]
        
        # Weight/Distance between last node and current node
        cost = self.dis(x1, y1, x2, y2)
        
        self.global_graph.add_edge(first_node, second_node, weight=cost)
            
    # Select two points to connect
    def connect_point(self, x, y):
        if self.first_selection == None:
            self.first_selection = self.get_closest_node(x, y)
        else:
            # Grab the second selection and begin connecting the two nodes
            self.second_selection = self.get_closest_node(x, y)
            
            self.add_weighted_edge(self.first_selection, self.second_selection)
            
            # Reset first and second selections
            self.first_selection = None
            self.second_selection = None
            
            self.point_mode = "Add"
            
    def get_closest_node(self, x, y):
        # Find closest node to passed point
        min_dist = 99999
        min_node = None
        for node in self.global_graph.nodes:
            # Position of each node in the graph
            node_posx = self.global_graph.node[node]['pos'][0]
            node_posy = self.global_graph.node[node]['pos'][1]
            
            dist = self.dis(node_posx, node_posy, x, y)
            print(str(dist))
            if dist < min_dist:
                min_node = node
                min_dist = dist
                print(str(min_node))
        
        return min_node
    
    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)
        w = 119
        a = 97
        s = 115
        c = 99
        r = 114
        g = 103
    
        stdscr.nodelay(True)
        rate = rospy.Rate(60) 
        stdscr.addstr(0,0,' A - Add a new point to the current path\n R - to remove a point (Recommended that you remove only the most recent node while auto-connect is on)\n C - Connect two points\n W - toggle auto-connecting nodes\nG - Save the graph(Will be named the current time)')

        while not rospy.is_shutdown():
            if self.display_graph is not None:
                self.display_rviz("/map")
            
            keyval = stdscr.getch()

            if keyval == self.prev_key:
                continue
            elif keyval == s:
                self.point_mode = "Select"
            elif keyval == a:
                self.point_mode = "Add"
            elif keyval == w:
                self.auto_connect = not (self.auto_connect)
                print( 'Auto-Conncect: ' + str(self.auto_connect))
            elif keyval == c:
                self.point_mode = "Connect"
            elif keyval == r:
                self.point_mode = "Remove"
            elif keyval == g:
                rospy.loginfo("Saving graph")
                g_name = "Graph: " + str(datetime.datetime.now()) + ".gml"
                nx.write_gml(self.global_graph, g_name)
                rospy.loginfo("Graph saved as: " + g_name)
            
            self.prev_key = keyval
            rate.sleep()
            
    def dis(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def display_rviz(self, frame):
        local_display = self.display_graph
        
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
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0

            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8

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
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.points = points

            marker.scale.x = 0.3
            marker.scale.y = 0.6
            marker.scale.z = 0

            self.display_pub.publish(marker)
            i += 1
        

if __name__ == "__main__":
    try:
        PathCreation()
    except rospy.ROSInterruptException:
        pass
