#!/usr/bin/env python

import rospy
import math
#from threading import Thread, Lock
import pid, gps_util, waypoint_handler
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from navigation_msgs.msg import EmergencyStop, LatLongPoint, WaypointsArray, VelAngle 
max_speed = 2.2352 #5 mph converted to m/s (temporary max)
tolerance = 2.0
hertz = 10 #rospy execution cycles/s
class TheOvermind(object):

    def __init__(self):
        global hertz
        global tolerance
        
        self.odom = None
        self.tolerance = tolerance #allowed imprecision for reaching a waypoint
        self.current_goal = None #current goal from list of goals
        self.vel_curr = 0.0 #current velocity
        self.angle_curr = 0.0 #current wheel angle
        self.kill = False
        self.vel_angle = VelAngle()
        #util classes
        self.waypoints = waypoint_handler.WaypointHandler(self.tolerance)
        self.angle_pid_controller = pid.PID(1,0,0,0,0)
        self.vel_pid_controller = pid.PID(1,0,0,0,0)
        #publishers                                    
        self.vel_angle_p = rospy.Publisher('/vel_angle', VelAngle, queue_size=10)
        #subscribers	
        self.odom_s = rospy.Subscriber('/pose_and_speed', Odometry, self.odom_callback, queue_size=10)
        self.point_cloud_s = rospy.Subscriber('/2d_point_cloud', PointCloud2, self.point_cloud_callback, queue_size=10) 
        self.killswitch_s = rospy.Subscriber('/stop', EmergencyStop, self.killswitch_callback, queue_size=10) 
        self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback, queue_size=10) 
        rospy.init_node('the_overmind', anonymous=False)
        self.control() 

    ''' main event loop '''
    def control(self):
        global hertz
        rate = rospy.Rate(hertz) # 10hz
        while not rospy.is_shutdown():
            #if killswitch, publish 0 to vel_angle's vel_curr, and leave the angle the same	
            self.controller_handler()
            self.vel_angle_p.publish(self.vel_angle)
            rate.sleep()
    
    def odom_callback(self, msg):
        self.odom = msg
        self.vel_angle.vel_curr = msg.twist.twist.linear.x
        #if the final goal has not been reached, get the next goal
        if not self.waypoints.update_pos(msg, tolerance):
            self.current_goal = self.waypoints.get_goal()
        else:
            self.current_goal = None

    ''' set a flag to true if emergency stop is activated '''		
    def killswitch_callback(self, msg):
        if msg.emergency_stop:
            self.kill = True

    def waypoints_callback(self, msg):
        #replace the waypoints array with new data
        self.waypoints.set_points(msg.waypoints)
        self.current_goal = get_goal()

    ''' handles controller related things in the main loop '''
    def controller_handler(self):
        #distance and angle from the next point
        dist = self.waypoints.distance_from_next()
        angle = self.waypoints.angle_from_next()
        self.vel_curr = self.vel_pid_controller.update_PID(dist or 0)
        self.angle_curr = self.angle_pid_controller.update_PID(angle or 0)
	
    def is_approx(actual, expected, tolerance):
        return expected - tolerance < actual < expected + tolerance
 
    def point_cloud_callback(self, msg):
        pass

if __name__ == '__main__':
    try:
       TheOvermind()
    except rospy.ROSInterruptException:
       pass
