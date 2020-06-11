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
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker

class PathCreation(object):
    def __init__(self):
        rospy.init_node('path_creation_tool')
        self.display_pub = rospy.Publisher('/path_display', Marker, queue_size=10)
        
        # Listen for cart position changes
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size=10)
        
        # Listen for RViz clicks
        self.point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        
        # Modes
        self.point_mode = "Add" # Select, Add, Remove, Connect ...a point
        self.road_type = "one_way" # One way single lane road (one_way), Two way single lane road (two_way), Two Lane road opposing directions(two_lane)
        
        # Auto-build the map
        self.auto_build = False

        # When adding new nodes, auto-connect previous node to new node
        self.auto_connect = True
        
        # Most recent cart position
        self.recent_pos = None

        # Debounce for keyboard input
        self.prev_key = 1

        # Keep track of global stae of graph(e.g. most recently placed node)
        self.global_graph = nx.DiGraph()
        self.last_node = None
        self.selected_node = None
        self.node_count = 0
        
        # For node selection 
        self.first_selection = None
        self.second_selection = None
        
        self.display_graph = None
        
        # How many meters between points when auto-build map is toggled on
        self.AUTO_POINT_GAP = 2
        
        #Enter edit mode if there is an existing file provided as an argument, disable auto-tools
        if len(sys.argv) > 1:
            self.auto_connect = False
            self.auto_build = False
            file_name = sys.argv[1]
            print("Loading: " + file_name)
            self.global_graph = nx.read_gml(file_name)
            self.display_graph = self.global_graph
            print("Loaded, keep in mind auto-connect, auto-build, should remain toggled off lest you want to unleash the kraken.\n (Basically don't press b or c)\n")
        curses.wrapper(self.get_input)
        
        
    # Upon receiving a clicked point from RViz, or being called from another method process the point
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
    
    # On receipt of new cart position, check spacing between current point and last
    def pose_callback(self, msg):
        current_pos = PointStamped()
        current_pos.point.x = msg.pose.position.x
        current_pos.point.y = msg.pose.position.y
        
        if self.recent_pos is None:
            self.recent_pos = current_pos
        
        # Place a new point down every specified amount of meters
        if self.dis(self.recent_pos.point.x, self.recent_pos.point.y, current_pos.point.x, current_pos.point.y) > self.AUTO_POINT_GAP:
            self.recent_pos = current_pos
            self.point_callback(current_pos)
        
    # Add point to the graph
    def add_point(self, x, y):
        # Create a rightside node representing right lane, also plays role of being the center in a one lane secnario
        node_name_r = 'R_Node:' + str(self.node_count)
        self.global_graph.add_node(node_name_r, pos=[x, y])
        
        node_name_l = 'L_Node' + str(self.node_count)
        #self.global_graph.add_node()
            
            
        #Connect previous node to current node
        if self.node_count > 0 and self.auto_connect:
            self.add_weighted_edge(self.last_node_right, node_name_r)
            # Create cycle between last and current node if this road is a one lane two way
            if self.road_type == "two_way":
                self.add_weighted_edge(self.node_count, self.last_node_right)
            elif self.road_type == "two_lane":
                #self.global_graph.add_node(self.node_name_l, pos=[])
                print("Two Laner")
                
        
        self.last_node_right = node_name_r
        self.node_count += 1
        
    def remove_point(self, x, y):
        # TODO checking for empty graph here
        # Find the closest node to where the user clicked
        min_node = get_closest_node(x, y)
        
        self.node_count -= 1
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
            
            
    def get_closest_node(self, x, y):
        # Find closest node to passed point
        min_dist = 99999
        min_node = None
        for node in self.global_graph.nodes:
            # Position of each node in the graph
            node_posx = self.global_graph.node[node]['pos'][0]
            node_posy = self.global_graph.node[node]['pos'][1]
            
            # Get the distance between the current node and
            dist = self.dis(node_posx, node_posy, x, y)
            if dist < min_dist:
                min_node = node
                min_dist = dist
        
        return min_node
    
    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)
        w = 119
        a = 97
        s = 115
        b = 98
        c = 99
        r = 114
        g = 103
    
        stdscr.nodelay(True)
        rate = rospy.Rate(60) 
        stdscr.addstr(0,0,""" A - Add a new point to the current path\n 
                      R - to remove a point (Recommended that you remove only the most recent node while auto-connect is on)\n 
                      C - Connect two points\n 
                      W - toggle auto-connecting nodes\n 
                      G - Save the graph(Will be named the current time)\n
                      B - Drive the cart around and auto-build a map""")

        while 1:
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
                print( 'Auto-Connect: ' + str(self.auto_connect) + "\n")
            elif keyval == c:
                self.point_mode = "Connect"
            elif keyval == r:
                self.point_mode = "Remove"
            elif keyval == g:
                rospy.loginfo("Saving graph")
                g_name = "Graph: " + str(datetime.datetime.now()) + ".gml"
                nx.write_gml(self.global_graph, g_name)
                rospy.loginfo("Graph saved as: " + g_name)
            elif keyval == b:
                self.auto_build = not (self.auto_build)
                print("Map auto-build set to: " + str(self.auto_build) + "\n")
                
            print("Current Point Mode: " + self.point_mode + "\n")
            
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
            marker.lifetime = rospy.Duration.from_sec(0.3)

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
