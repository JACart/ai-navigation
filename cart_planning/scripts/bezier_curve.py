#!/usr/bin/env python

import rospy
import math

from navigation_msgs.msg import EmergencyStop, Obstacle, ObstacleArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

"""
Class that takes the carts current position and a variable distance.
Calculates a bezier curve given those two parameters.
If an object is on the carts trajectory and close an emergency stop message is sent
If an object is on the trajectory and far enough way to create a new path then the cart is rerouted (TODO)
"""
class Bezier_Curve(object):

    def __init__(self, distance):
        self.pose_sub = rospy.Subscriber('/pose_and_speed', Odometry,
                                            self.pose_callback, queue_size=10)
        self.obstacles_sub = rospy.Subscriber('/obstacles', ObstacleArray, self.obstacles_callback, queue_size=10)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        
        self.position = None
        self.goal = None
        self.angles = None
        self.x2 = None
        self.distance = distance
        self.coefficient = None
        self.obstacles = None
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.obstacles is not None:
                self.determining__endpoint()
                self.control_point()
                self.calculating_coefficient
                self.determine_collision
            r.sleep()

        
    def pose_callback(self, message):
        self.position = message.pose.pose

    def obstacles_callback(self, message):
        self.obstacles = message.obstacles
    
    """
    Looks at the position of the obstacle and determines if its on the cart trajectory.
    Either sends a message to stop the cart or reroutes the cart if needed.
    """
    def determine_collision(self):
        for obstacle in self.obstacles:
            dist_to_cart = math.sqrt((obstacle.pos.point.x - self.position.position.x)**2 + (obstacle.pos.point.y - self.position.position.y)**2)
            #(1-t)^2*p0 + 2(1-t)*t*p1 +t^2*p2
            calc_y = (1-self.coefficient.y)**2*self.position.position.y + 2*(1-self.coefficient.y)*self.coefficient.y*self.x2.y+ self.coefficient.y**2+self.goal.y
            on_path = calc_y - 1 <= obstacle.pos.position.y <= calc_y + 1 
            if dist_to_cart < 3 and on_path:
                #send an emergency stop
                stop_msg = EmergencyStop()
                stop_msg.emergency_stop = True
                self.emergency_stop_pub.publish(stop_msg)
            #TODO edit this distance
            elif dist_to_cart < 9 and on_path:
                #TODO reroute
                pass
    
    #Calculates the endpoint of the bezier curve
    def determining__endpoint(self):
        adj = math.cos(self.angles[3])*self.distance
        opp = math.sin(self.angles[3])*self.distance
        self.goal = Point()
        self.goal.x = self.position.position.x + adj
        self.goal.y = self.position.position.y + opp

    #Calculates the coefficient of the bezier curve
    def calculating_coefficient(self):
        self.coefficient = Point()
        self.coefficient.x = (self.x2.x + math.sqrt(self.x2.x**2 - 4*self.position.position.x*self.goal.x))/(2*self.position.position.x)
        self.coefficient.y = (self.x2.y + math.sqrt(self.x2.y**2 - 4*self.position.position.y*self.goal.y))/(2*self.position.position.y)

    #Calculates the control point of the bezier curve
    def control_point(self):
        quat = (self.position.orientation.x, self.position.orientation.y, 
                self.position.orientation.z, self.position.orientation.w)
        self.angles = tf.euler_from_quaternion(quat)
        
        ax1 = math.cos(self.angles[3])*self.distance*.33
        ay1 = math.sin(self.angles[3])*self.distance*.33

        self.x2 = Point()
        x2.x = self.position.position.x + ax1*self.position.position.x
        x2.y = self.position.position.y + ay1*self.position.position.y
        
        self.x3 = Point()
        x2.x = self.goal.position.x - ax2*self.goal.position.x
        x2.y = self.goal.position.y - ay2*self.goal.position.y        
