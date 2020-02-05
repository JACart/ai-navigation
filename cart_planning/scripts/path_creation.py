#!/usr/bin/env python 
import curses
import rospy
import sys
import time
import math
import copy
import networkx as nx

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
        
        #When adding new nodes, auto-connect previous node to new node
        self.auto_connect = True
        
        curses.wrapper(self.get_input)
        
        
        
    def point_callback(self, msg):
        if self.point_mode is "Add":
            rospy.loginfo("Adding node: " + str(self.node_count))
            node_name = 'Node:' + str(self.node_count)
            node_x = msg.point.x
            node_y = msg.point.y
            self.global_graph.add_node(self.node_count, name=node_name, pos=[node_x, node_y])
            
            
            #Connect previous node to current node
            if self.last_node >= 0 and self.auto_connect:
                x1 = self.global_graph.node[self.last_node]['pos'][0]
                y1 = self.global_graph.node[self.last_node]['pos'][1]
                x2 = self.global_graph.node[self.node_count]['pos'][0]
                y2 = self.global_graph.node[self.node_count]['pos'][1]
                
                # Weight/Distance between last node and current node
                cost = self.dis(x1, y1, x2, y2)
                
                self.global_graph.add_edge(self.last_node, self.node_count, weight=cost)
            
            self.last_node = self.node_count
            self.node_count += 1
            
            
        elif self.point_mode is "Remove":
            rospy.loginfo("Removing point")
        else:
            rospy.loginfo("Selecting point")

    def get_closest_node(self, PointStamped):
        # Find closest node to passed point
        rospy.loginfo("Get closest point")
    
    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)
        w = 119
        a = 97
        s = 115
        d = 100
        x = 120
        y = 121
    
        stdscr.nodelay(True)
        rate = rospy.Rate(60) 
        stdscr.addstr(0,0,'S to select a point that already exists, A to add a new point to the current path, R to remove most recent point\n C to connect two points. W to toggle auto-connecting nodes')

        while not rospy.is_shutdown():
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
            
            self.prev_key = keyval
            rate.sleep()
            
    def dis(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
    def display_rviz(self, frame):
        display_graph = copy.deepcopy(self.global_graph)
        
        i = 0
        for node in display_graph.nodes:
            x = display_graph.node[node]['pos'][0]
            y = display_graph.node[node]['pos'][1]
            
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

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            self.display_pub.publish(marker)
            i += 1
            
        for edge in display_graph.edges:
            first_node = display_graph.node[edge[0]]
            second_node = display_graph.node[edge[1]]
            
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

            marker.scale.x = 0.1
            marker.scale.y = 0.3
            marker.scale.z = 0

            self.display_pub.publish(marker)
            i += 1
        

if __name__ == "__main__":
    try:
        PathCreation()
    except rospy.ROSInterruptException:
        pass
