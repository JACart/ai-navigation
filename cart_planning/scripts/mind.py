#!/usr/bin/env python

import math
import gps_util
import geometry_util
import rospy
from navigation_msgs.msg import WaypointsArray, VelAngle, LocalPointsArray, VehicleState
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Float32, String, Int8, Bool
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, Pose, Twist
from visualization_msgs.msg import Marker
import tf.transformations as tf

import cubic_spline_planner
import pure_pursuit

'''
This class contains the code to track and fit a path
for the cart to follow
'''
class Mind(object):
    def __init__(self):
        rospy.init_node('Mind')

        self.odom = Odometry()
        self.debug = False
        #Our current velocity (linear x value)
        self.gTwist = Twist()
        
        #Our current position in local coordinates
        self.gPose = Pose()

        self.meters = 10.0
        self.seconds = 3.6

        self.delay_print = 0

        self.global_speed = self.meters / self.seconds  # [m/s]

        self.navigating = False
        self.new_path = False
        self.path_valid = False
        self.google_points = []
        self.rp_dist = 99999999999
        self.stop_thresh = 5 #this is how many seconds an object is away

        #waypoints are points coming in from the map
        self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray,
                                            self.waypoints_callback, queue_size=10)
                                            
        self.local_sub = rospy.Subscriber('/global_path', LocalPointsArray,
                                            self.localpoints_callback, queue_size=10)
                                            
        self.odom_sub = rospy.Subscriber('/pose_and_speed', Odometry,
                                         self.odom_callback, queue_size=10)
                                         
        self.rp_distance_sub = rospy.Subscriber('/rp_distance', Float32,
                                                self.rp_callback, queue_size=10)
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
                                                
        self.twist_sub = rospy.Subscriber('/estimate_twist', TwistStamped, self.twist_callback, queue_size = 10)
        self.pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.pose_callback, queue_size = 10)
        self.param_sub = rospy.Subscriber('realtime_param_change', Int8, self.param_callback, queue_size = 10)
        #publishes points that are now in gps coordinates
        self.vehicle_state_pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size=10)
        self.points_pub = rospy.Publisher('/points', Path, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10, latch=True)
        self.motion_pub = rospy.Publisher('/nav_cmd', VelAngle, queue_size=10)
        self.target_pub = rospy.Publisher('/target_point', Marker, queue_size=10)
        self.target_twist_pub = rospy.Publisher('/target_twist', Marker, queue_size=10)
        
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.new_path: #when a new path is recieved by the callback run create_path
                rospy.loginfo("Starting Create Path")
                self.path_valid = True
                self.new_path = False
                self.create_path()
            rate.sleep()

    def param_callback(self, msg):
        self.meters += msg.data
        self.global_speed = self.meters / self.seconds
        rospy.loginfo("Meters/Second: " + str(self.global_speed))

    def odom_callback(self, msg):
        self.odom = msg
    
    def twist_callback(self, msg):
        self.gTwist = msg.twist
        
    def pose_callback(self, msg):
        self.gPose = msg.pose

    def rp_callback(self, msg):
        #used to stop the vehicle if objects are within a certain distance of the cart
        if msg.data <= 0.5:
            self.rp_dist = 99999999
        else:
            self.rp_dist = msg.data
    
    def debug_callback(self, msg):
        self.debug = msg.data
        rospy.loginfo(self.debug)

    def localpoints_callback(self, msg):
        self.google_points = []
        rospy.loginfo('Before looping')
        for local_point in msg.localpoints:
            self.google_points.append(local_point.position)
        rospy.loginfo('Creating Mind Path')
        #path_valid being set to false will end the previous navigation and new_path being true will trigger the starting of the new path
        self.path_valid = False
        self.new_path = True


    #Converts waypoints into points in gps coordinates as opposed to the map
    def waypoints_callback(self, msg):
        for gps_point in msg.waypoints:
            point = gps_util.get_point(gps_point)
            self.google_points.append(point)

        self.create_path()
    '''
    Creates a path for the cart with a set of google_points
    Adds 15 more points between the google points
    Intermediate points are added for a better fitting spline
    '''
    def create_path(self):
        #creates 15 intermediate points
        google_points_plus = geometry_util.add_intermediate_points(self.google_points, 15.0)

        ax = []
        ay = []

        #Creates another path to add intermediate points for a better fitting spline
        extra_points = Path()
        extra_points.header = Header()
        extra_points.header.frame_id = '/map'
        
        last_index = 0
        target_ind = 0
        
        #Creates a list of the x's and y's to be used when calculating the spline
        for p in google_points_plus:
            extra_points.poses.append(create_pose_stamped(p))
            ax.append(p.x)
            ay.append(p.y)

        self.points_pub.publish(extra_points)

        if len(ax) > 2:
            cx, cy, cyaw, ck, cs = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
            #Create path object for the cart to follow
            path = Path()
            path.header = Header()
            path.header.frame_id = '/map'

            for i in range(0, len(cx)):
                curve_point = Point()
                curve_point.x = cx[i]
                curve_point.y = cy[i]
                path.poses.append(create_pose_stamped(curve_point))
            
            self.path_pub.publish(path)

            current_state = VehicleState()
            current_state.is_navigating = True
            self.vehicle_state_pub.publish(current_state)
            
            target_speed = self.global_speed

            # initial state
            pose = self.gPose
            twist = self.gTwist

            quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            angles = tf.euler_from_quaternion(quat)
            initial_v = twist.linear.x
            #TODO state has to be where we start
            state = State(x=pose.position.x, y=pose.position.y, yaw=angles[2], v=initial_v)

            last_index = len(cx) - 1
            time = 0.0
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            v = [state.v]
            t = [0.0]
            target_ind = pure_pursuit.calc_target_index(state, cx, cy, 0)

             #continue to loop while we have not hit the target
            while last_index > target_ind and self.path_valid and not rospy.is_shutdown():
                target_speed = self.global_speed            
                ai = target_speed#pure_pursuit.PIDControl(target_speed, state.v)
                di, target_ind = pure_pursuit.pure_pursuit_control(state, cx, cy, target_ind)
                
                #publish our desired position
                mkr = create_marker(cx[target_ind], cy[target_ind], '/map')
                self.target_pub.publish(mkr)

                #publish an arrow with our twist
                arrow = create_marker(0, 0, '/base_link')
                arrow.type = 0 #arrow
                arrow.scale.x = 2.0
                arrow.scale.y = 1.0
                arrow.scale.z = 1.0
                arrow.color.r = 1.0
                arrow.color.g = 0.0
                arrow.color.b = 0.0

                quater = tf.quaternion_from_euler(0, 0, di)
                arrow.pose.orientation.x = quater[0]
                arrow.pose.orientation.y = quater[1]
                arrow.pose.orientation.z = quater[2]
                arrow.pose.orientation.w = quater[3]
                self.target_twist_pub.publish(arrow)

                state = self.update(state, ai, di)

                x.append(state.x)
                y.append(state.y)
                yaw.append(state.yaw)
                v.append(state.v)
                t.append(time)
        else:
            self.path_valid = False
            rospy.logwarn("It appears the cart is already at the destination")
            
        #Check if we've reached the destination, if so we should let the network client know
        rospy.loginfo("Done navigating")
        current_state = VehicleState()
        current_state.is_navigating = False
        if self.path_valid:
            current_state.reached_destination = True
            rospy.loginfo("Reached Destination")
        else:
            current_state.reached_destination = False
            rospy.loginfo("Destination not reached. There may be no path to get to the destination or the cart is already there.")
        
        self.vehicle_state_pub.publish(current_state)
        msg = VelAngle()
        msg.vel = 0
        msg.angle = 0
        msg.vel_curr = 0
        self.motion_pub.publish(msg)
        


    '''
    Updates the carts position by a given state and delta
    '''
    def update(self, state, a, delta):

        pose = self.gPose
        twist = self.gTwist
        current_spd = twist.linear.x
        msg = VelAngle()
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 50
                rospy.loginfo("Target Speed: " + str(a))
                rospy.loginfo("Current Speed: " + str(current_spd))
        msg.vel = a #Speed we want from pure pursuit controller
        msg.angle = (delta*180)/math.pi
        msg.vel_curr = current_spd
        self.motion_pub.publish(msg)

        state.x = pose.position.x
        state.y = pose.position.y

        quat = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = tf.euler_from_quaternion(quat)

        state.yaw = angles[2]

        state.v = twist.linear.x

        return state


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def create_pose_stamped(point):
    stamped = PoseStamped()
    stamped.header = Header()
    stamped.header.frame_id = '/map'
    stamped.pose.position = point
    return stamped

def create_marker(x, y, frame_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = 1 #cube
    marker.action = 0 #add
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    return marker

if __name__ == "__main__":
    try:
        Mind()
    except rospy.ROSInterruptException:
        pass
