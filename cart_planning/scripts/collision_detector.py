#!/usr/bin/env python

'''
Collision Detector node, takes in obstacle data from the obstacle detector endpoint and makes decisions based on the obstacles.

Important Notes Before Continuing:
-Experimental at the moment, very 'prototypey'
-Math is performed in relation to the base_link frame,
    which based on the robot description is rotated half pi radians (counter-clockwise given conventions)
'''

import socket
import rospy
import math
import tf
import numpy as np

from std_msgs.msg import Header, Float32
from visualization_msgs.msg import MarkerArray
from navigation_msgs.msg import ObstacleArray, Obstacle, EmergencyStop, VelAngle
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker

class CollisionDetector(object):

    def __init__(self):
        self.stop = False
        rospy.init_node('collision_detector')

        self.t = tf.TransformListener()

        self.cur_obstacles = None
        self.cur_pos = None
        self.target_pos = None

        self.lookahead_local = None

        self.requested_steering_angle = 0

        # Vehicle Kinematics
        self.vehicle_width = rospy.get_param('vehicle_width')
        self.vehicle_length = rospy.get_param('vehicle_length')
        self.wheel_base = rospy.get_param('wheel_base')
        self.front_axle_track = rospy.get_param('front_axle_track')
        self.rear_axle_track = rospy.get_param('rear_axle_track')
        self.tire_width = rospy.get_param('tire_width')
        self.cur_speed = 0.1
        self.stopped = False
        self.cleared_confidence = 0

        # Vehicle Corners (Wheel Positions)
        self.front_right_corner = [0, 0]
        self.front_left_corner = [0, 0]
        self.rear_left_corner = [0, 0]
        self.rear_right_corner = [0, 0]
        self.front_axle_center = [0, 0]

        # Collision Bounds
        self.inner_radius = 0
        self.outer_radius = 0
        self.circle_center = [0, 0]
        self.right_turn = False

        # Minimum allowable distance from front of cart to intercept obstacle before emergency stopping
        self.min_obstacle_dist = rospy.get_param('min_obstacle_dist')
        # Minimum allowable transit time to an obstacle allowed before emergency stopping
        self.min_obstacle_time = rospy.get_param('min_obstacle_time')

        self.safe_obstacle_dist = rospy.get_param('safe_obstacle_dist')
        self.safe_obstacle_time = rospy.get_param('safe_obstacle_time')

        self.obstacle_sub = rospy.Subscriber('/obstacles', ObstacleArray, self.obstacle_callback, queue_size=10)
        self.pos_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.position_callback, queue_size=10)
        self.angle_sub = rospy.Subscriber('/nav_cmd', VelAngle, self.angle_callback, queue_size=10)
        self.cur_speed_sub = rospy.Subscriber('/estimated_vel_mps', Float32, queue_size=10)
        
        self.display_pub = rospy.Publisher('/corner_display', Marker,queue_size=10)
        self.stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        self.gentle_stop_pub = rospy.Publisher('/request_stop', EmergencyStop, queue_size=10)
        self.display_boundary_pub = rospy.Publisher('/boundaries', Marker, queue_size=10)
        self.display_array = rospy.Publisher('/boundaries_array', MarkerArray, queue_size=100)
        self.collision_pub = rospy.Publisher('/collision_pub', MarkerArray, queue_size=100)

        self.obtain_corners()

        # Run collision detection 30 times a second
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.calc_arcs(self.requested_steering_angle)
            if self.cur_obstacles is not None:
                # Calculate the inner and outer radius for arcs and the center of their circ
                self.determine_collision()
            r.sleep()

    def angle_callback(self, msg):
        self.requested_steering_angle = msg.angle

    def speed_callback(self, msg):
        self.cur_speed = msg.data
        if self.cur_speed < 0.1:
            self.cur_speed = 0.1

    def obtain_corners(self):
        """ This could be simplified to simply obtaining the position using the tf library
        Point of experimenting in the next couple days, not sure which is more flexible yet.
        """

        self.rear_left_corner = [-(self.wheel_base/2), ((self.front_axle_track/2) + (self.tire_width/2))]
        self.rear_right_corner = [-(self.wheel_base/2), -((self.front_axle_track/2) + (self.tire_width/2))]
        self.front_right_corner = [(self.wheel_base/2), -((self.rear_axle_track/2) + (self.tire_width/2))]
        self.front_left_corner = [(self.wheel_base/2), ((self.rear_axle_track/2) + (self.tire_width/2))]
        self.front_axle_center = [(self.wheel_base/2), 0]

    def calc_arcs(self, steering_angle):
        """ Calculates the Inner turn radius arc and outer turn radius arc and the 
        center point the two arcs share.

        Args:
            steering_angle(Float): Steering angle of the cart in degrees
        """
        # Used for RViz display
        marker_array = MarkerArray()
        
        # If steering angle is small/near 0 apply a small angle in the same direction
        if steering_angle < 1 and steering_angle >= 0:
            steering_angle = 1
        elif steering_angle > -1 and steering_angle <= 0:
            steering_angle = -1
        
        steering_angle = math.radians(steering_angle)

        # Calculate inner radius, which is used to find the circle center for the arcs
        self.inner_radius = self.wheel_base/math.tan(steering_angle)
        
        # Turning left
        if steering_angle > 0:
            # calculate outer arc radius and center of the circle for both arcs
            self.outer_radius = self.inner_radius + self.vehicle_width
            self.circle_center = [self.rear_left_corner[0], self.rear_left_corner[1] + self.inner_radius]

            # Create markers for RViz display
            inner_arc = self.display_arc(self.circle_center, self.inner_radius, id=10)
            outer_arc = self.display_arc(self.circle_center, self.outer_radius, id=11)
            marker_array.markers.append(inner_arc)
            marker_array.markers.append(outer_arc)

            self.right_turn = False
            

        # Turning right
        elif steering_angle < 0:
            # calculate outer arc radius and center of the circle for both arcs
            self.outer_radius = self.inner_radius - self.vehicle_width
            self.circle_center = [self.rear_right_corner[0], self.rear_right_corner[1] + self.inner_radius]

            # Create markers for RViz display
            inner_arc = self.display_arc(self.circle_center, self.inner_radius, id=15, right_turn=True)
            outer_arc = self.display_arc(self.circle_center, self.outer_radius, id=18, right_turn=True)
            marker_array.markers.append(inner_arc)
            marker_array.markers.append(outer_arc)

            self.right_turn = True
        
        self.display_array.publish(marker_array)
        
    def determine_collision(self):
        """ Simply determines if an obstacle is a potential collision
        """
        # For displaying hazardous obstacles
        collision_array = MarkerArray()
        display = None

        # Control for undoing a stop
        clear_path = True

        cur_obstacle_list = self.cur_obstacles
        for obstacle in cur_obstacle_list:
            obstacle_size = 2 * obstacle.radius

            # The distance/radius of the circle center to the obstacle center
            circle_obstacle_dist = self.distance(obstacle.pos.point.x, obstacle.pos.point.y, 
            self.circle_center[0], self.circle_center[1])
            # Determine if the obstacle is within bounds of the inner and outer arc
            potential_collision = (abs(self.inner_radius) - obstacle_size) < circle_obstacle_dist < (abs(self.outer_radius) + obstacle_size)


            self.display_circle("/base_link", self.front_axle_center, 24, 2, z=1)

            if potential_collision:
                # Prepare an emergency stop message
                stop_msg = EmergencyStop()
                stop_msg.emergency_stop = True
                stop_msg.sender_id.data = "collision_detector"

                # Calculate distance from front of cart to obstacle
                distance = self.distance(obstacle.pos.point.x, obstacle.pos.point.y, 
                self.front_axle_center[0], self.front_axle_center[1])
                # Calculate rough time to obstacle impact (seconds)
                impact_time = distance/self.cur_speed
                if distance < self.min_obstacle_dist or impact_time < self.min_obstacle_time:
                    clear_path = False
                    self.cleared_confidence = 0
                    if not self.stopped:
                        self.stopped = True
                        self.stop_pub.publish(stop_msg)
                        rospy.logwarn("Requesting a stop due to possible collision")
                    # Show a red obstacle, an obstacle worth stopping for
                    display = self.show_colliding_obstacle(obstacle.pos.point.x, obstacle.pos.point.y, color=0.0)
                elif distance < self.safe_obstacle_dist or impact_time < self.safe_obstacle_time:
                    #Temporay code duplication
                    clear_path = False
                    self.cleared_confidence = 0
                    if not self.stopped:
                        self.stopped = True
                        self.gentle_stop_pub.publish(stop_msg)
                        rospy.logwarn("Requesting a stop due to possible collision")
                    # Show a red obstacle, an obstacle worth stopping for
                    display = self.show_colliding_obstacle(obstacle.pos.point.x, obstacle.pos.point.y, color=0.0)
                else:
                    # Show a yellow obstacle, obstacle that has potential
                    display = self.show_colliding_obstacle(obstacle.pos.point.x, obstacle.pos.point.y)
            
            if display is not None:
                collision_array.markers.append(display)
            
        # If a run detects a clear path, but we are still stopped. allow nav to continue, and we have confidence is clear (15 spins/3 seconds given rate of 5 hz)
        if clear_path:
            self.cleared_confidence += 1
            if self.stopped and self.cleared_confidence >= 15:
                self.cleared_confidence = 0
                rospy.logwarn("Clearing collision, continuing navigation")
                self.stopped = False
                stop_msg = EmergencyStop()
                stop_msg.emergency_stop = False
                stop_msg.sender_id.data = "collision_detector"
                self.stop_pub.publish(stop_msg)
                self.gentle_stop_pub.publish(stop_msg)

        self.collision_pub.publish(collision_array)

    def show_colliding_obstacle(self, x, y, color=1.0):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "/base_link"
        
        marker.type = marker.CUBE
        marker.action = 0
        
        marker.color.r = 1.0
        marker.color.g = color
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration.from_sec(0.2)
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.2
        
        return marker 

    def display_circle(self, frame, corner, id, radius, color=0.0, z=0.05):
        """ Create circle marker for RViz
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = frame

        marker.ns = "Object_NS"
        marker.id = id
        marker.type = Marker.CYLINDER
        marker.action = 0
        marker.color.r = color
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.1
        marker.lifetime = rospy.Duration.from_sec(0.1)

        marker.pose.position.x = corner[0]
        marker.pose.position.y = corner[1]
        marker.pose.position.z = 1

        radius = radius
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = z

        self.display_pub.publish(marker)      

    def obstacle_callback(self, msg):
        self.cur_obstacles = msg.obstacles

    def position_callback(self, msg):
        self.cur_pos = msg.pose
        
    def display_arc(self, circle, radius, id, right_turn=False):
        """ Create and return an arc for RViz using a Line Strip Marker

        Args:
            circle(Array): Tuple [x,y] of circle center
            radius(float): Radius for the circle
            id(Int): ID for the marker
            right_turn(Boolean): Whether the arc is being displayed for a right turn or not
        """
        bound_display = Marker()
        bound_display.header = Header()
        bound_display.id = id
        bound_display.lifetime = rospy.Duration(secs=0.033)
        bound_display.type = Marker.LINE_STRIP
        bound_display.header.frame_id = "/base_link"
        bound_display.scale.x = 0.2
        bound_display.color.r = 1
        bound_display.color.g = 1
        bound_display.color.b = 0
        bound_display.color.a = 1.0
        bound_display.action = Marker.ADD
        
        # Obtain the angle range necessary for a certain arc length in this case 20
        ang = 20/(2*radius)
        
        # Setup arc display range
        loop_range = np.arange(-(math.pi/2), (-(math.pi/2)) + ang, .10)
        if right_turn:
            loop_range = np.arange((math.pi/2) + ang, math.pi/2, .10)

        # Obtain the points along the arc on the circle
        for ang in loop_range:
            arc_point = Point()
            if not right_turn:
                arc_point.x = circle[0] +  (radius * math.cos(ang))
                arc_point.y = circle[1] +  (radius * math.sin(ang))
            else:
                arc_point.x = circle[0] -  (radius * math.cos(ang))
                arc_point.y = circle[1] -  (radius * math.sin(ang))

            bound_display.points.append(arc_point)

        return bound_display


    def distance(self, x1, y1, x2, y2):
        dX = (x2 - x1)**2
        dY = (y2 - y1)**2

        return math.sqrt(dX + dY)


if __name__ == "__main__":
    try:
        CollisionDetector()
    except rospy.ROSInterruptException:
        pass

