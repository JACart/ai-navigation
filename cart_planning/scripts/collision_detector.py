#!/usr/bin/env python

'''
Collision Detector node, takes in obstacle data from the obstacle detector endpoint and makes decisions based on the obstacles.

Important Notes Before Continuing:
-Experimental at the moment
-Math is performed in relation to the base_link frame,
    which based on the robot description is rotated half pi radians (counter-clockwise given conventions)
'''

import socket
import rospy
import math
import tf
import numpy as np

from std_msgs.msg import Header
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

        # Vehicle Corners (Wheel Positions)
        self.front_right_corner = [0, 0]
        self.front_left_corner = [0, 0]
        self.rear_left_corner = [0, 0]
        self.rear_right_corner = [0, 0]

        # Collision Bounds
        self.inner_radius = 0
        self.outer_radius = 0
        self.circle_center = [0, 0]
        self.right_turn = False

        self.obstacle_sub = rospy.Subscriber('/obstacles', ObstacleArray, self.obstacle_callback, queue_size=10)
        self.pos_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.position_callback, queue_size=10)
        self.angle_sub = rospy.Subscriber('/nav_cmd', VelAngle, self.angle_callback, queue_size=10)

        self.display_pub = rospy.Publisher('/corner_display', Marker,queue_size=10)
        self.stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        self.display_boundary_pub = rospy.Publisher('/boundaries', Marker, queue_size=10)
        self.display_array = rospy.Publisher('/boundaries_array', MarkerArray, queue_size=100)
        self.collision_pub = rospy.Publisher('/collision_pub', Marker, queue_size=100)

        self.obtain_corners()

        # Run collision detection 30 times a second
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.cur_obstacles is not None:
                # Calculate the inner and outer radius for arcs and the center of their circle
                self.calc_arcs(self.requested_steering_angle)
                self.determine_collision()
            r.sleep()
    
    def angle_callback(self, msg):
        self.requested_steering_angle = msg.angle

    def obtain_corners(self):
        """ This could be simplified to simply obtaining the position using the tf library
        Point of experimenting in the next couple days, not sure which is more flexible yet.
        """

        self.rear_left_corner = [-(self.wheel_base/2), ((self.front_axle_track/2) + (self.tire_width/2))]
        self.rear_right_corner = [-(self.wheel_base/2), -((self.front_axle_track/2) + (self.tire_width/2))]
        self.front_right_corner = [(self.wheel_base/2), -((self.rear_axle_track/2) + (self.tire_width/2))]
        self.front_left_corner = [(self.wheel_base/2), ((self.rear_axle_track/2) + (self.tire_width/2))]

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
        cur_obstacle_list = self.cur_obstacles
        for obstacle in cur_obstacle_list:

            obstacle_size = 2 * obstacle.radius

            # The distance/radius of the circle center to the obstacle center
            circle_obstacle_dist = self.distance(obstacle.pos.point.x, obstacle.pos.point.y, 
            self.circle_center[0], self.circle_center[1])

            '''if self.right_turn:
                    rospy.logwarn("Inner: " + str(self.inner_radius) + " Outer: " + str(self.outer_radius) + " Object Radius: " + str(circle_obstacle_dist))'''

            # Determine if the obstacle is within bounds of the inner and outer arc
            potential_collision = (abs(self.inner_radius) - obstacle_size) < circle_obstacle_dist < (abs(self.outer_radius) + obstacle_size)
            #rospy.loginfo("Collision: " + str(potential_collision) + " Context: Inner Radius: " + str(self.inner_radius) + " Outer: " + str(self.outer_radius) + str(circle_obstacle_dist))
            # More conditions can be added here for determining collision criteria
            if potential_collision:
                rospy.logwarn("Looks like a potential collision")
                self.show_colliding_obstacle(obstacle.pos.point.x, obstacle.pos.point.y)


    def show_colliding_obstacle(self, x, y):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "/base_link"
        
        marker.type = marker.CUBE
        marker.action = 0
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration.from_sec(0.2)
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.2
        
        self.collision_pub.publish(marker)        

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
            #loop_range = np.arange(math.pi - ang, 3*(math.pi/2), .10)
            loop_range = np.arange(0, 2*(math.pi), .10)

        # Obtain the points along the arc on the circle
        for ang in loop_range:
            arc_point = Point()
            arc_point.x = circle[0] +  (radius * math.cos(ang))
            arc_point.y = circle[1] +  (radius * math.sin(ang))

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

