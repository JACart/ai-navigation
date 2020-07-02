#!/usr/bin/env python

'''
Local Planner node, takes in obstacle data from the obstacle detector endpoint and makes decisions based on the obstacles.
'''

import socket
import rospy
import math
import tf

from navigation_msgs.msg import ObstacleArray, Obstacle, EmergencyStop
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32, Point
from visualization_msgs.msg import Marker

class LocalPlanner(object):

    def __init__(self):
        self.stop = False
        rospy.init_node('local_planner')

        self.t = tf.TransformListener()

        self.cur_obstacles = None
        self.cur_pos = None
        self.target_pos = None

        self.lookahead_local = None

        self.cart_width = 0.5715 #Constant width of the front of the cart
        self.obstacle_sub = rospy.Subscriber('/obstacles', ObstacleArray, self.obstacle_callback, queue_size=10)
        self.pos_sub = rospy.Subscriber('/ndt_pose', PoseStamped, self.position_callback, queue_size=10)
        self.taget_sub = rospy.Subscriber('/target_point', Marker, self.target_callback, queue_size=10)

        self.line_pub = rospy.Publisher('/collision_line', Marker, queue_size=10)
        self.stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        self.boundingbox_pub = rospy.Publisher('/bb_viz', PolygonStamped, queue_size=10)
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.target_pos is not None:

                self.t.waitForTransform("/base_laser_link", "/base_link", rospy.Time(0), rospy.Duration(0.01))
                link_pos, left_quat = self.t.lookupTransform("/base_link", "/base_laser_link", rospy.Time())
                
                x1 = link_pos[0]
                y1 = link_pos[1]
                x2 = self.target_pos.position.x
                y2 = self.target_pos.position.y


                p1 = Point()
                p1.x = x1
                p1.y = y1
                p1.z = 0.5

                p2 = Point()
                p2.x = x2
                p2.y = y2
                p2.z = 0.5
        
                line_list = Marker()
                line_list.header.frame_id = "/base_link"
                line_list.header.stamp = rospy.Time(0)
                line_list.id = 100

                line_list.type = Marker.LINE_LIST
                line_list.points.append(p1)
                line_list.points.append(p2)

                self.line_pub.publish(line_list)

                for obstacle in self.cur_obstacles:
                    cx = obstacle.pos.point.x
                    cy = obstacle.pos.point.y
                    radius = obstacle.radius

                    if self.collision_check(x1, y1, x2, y2, cx, cy, radius):
                        stop_msg = EmergencyStop()
                        stop_msg.emergency_stop = True
                        stop_msg.sender_id = 6
                        self.stop_pub.publish(stop_msg)
                        rospy.loginfo("COLLISION DETECTED")
                    else:
                        stop_msg = EmergencyStop()
                        stop_msg.emergency_stop = False
                        stop_msg.sender_id = 6
                        self.stop_pub.publish(stop_msg)
                        rospy.loginfo("No Collision")
                    
            r.sleep()

    def obstacle_callback(self, msg):
        self.cur_obstacles = msg.obstacles

    def position_callback(self, msg):
        self.cur_pos = msg.pose

    def target_callback(self, msg):
        self.target_pos = msg.pose
        lookahead_global = PoseStamped()
        lookahead_global.header.frame_id = "/map"
        lookahead_global.pose = self.target_pos

        self.lookahead_local = self.t.transformPose("/base_link", lookahead_global)
        
    def collision_check(self, x1, y1, x2, y2, cx, cy, radius):
        radius += self.cart_width / 2 #Add buffer to circle radius

        inside = self.pointCircle(x1, y1, cx, cy, radius)
        inside2 = self.pointCircle(x1, y2, cx, cy, radius)

        if inside or inside2:
            return True
        
        #Line length
        dX = x1 - x2
        dY = y1- y2
        len = math.sqrt( (dX**2) + (dY**2) )

        #Dot product of line and circle
        dot = ( ((cx-x1)*(x2-x1)) + ((cy-y1)*(y2-y1)) ) / math.pow(len, 2)

        closestX = x1 + (dot * (x2-x1))
        closestY = y1 + (dot * (y2-y1))

        on_segment = self.linePoint(x1, y1, x2, y2, closestX, closestY)

        #Ensure point on line
        if not on_segment:
            return False

        dX = closestX - cx
        dY = closestY - cy

        dist = math.sqrt( (dX**2) + (dY**2) )

        if dist <= radius:
            return True
        
        return False

    def pointCircle(self, px, py, cx, cy, radius):
        dX = px - cx
        dY = py - cy

        dist = math.sqrt((dX**2) + (dY**2))

        if dist <= radius:
            return True

        return False

        
    def linePoint(self, x1, y1, x2, y2, px, py):
        d1 = self.distance(px, py, x1, y1)
        d2 = self.distance(px, py, x2, y2)

        line_len = self.distance(x1, y1, x2, y2)

        float_buff = 0.01

        if((d1 + d2 >= (line_len - float_buff)) and ((d1+d2) <= line_len+float_buff) ):
            return True

        return False
        
    def distance(self, x1, y1, x2, y2):
        dX = (x2 - x1)**2
        dY = (y2 - y1)**2

        return math.sqrt(dX + dY)

    def calcBounds(self):
        display_bb = PolygonStamped()
        self.t.waitForTransform("/front_left_caster", "/base_link", rospy.Time(0), rospy.Duration(0.01))
        self.t.waitForTransform("/front_right_caster", "/base_link", rospy.Time(0), rospy.Duration(0.01))
        left_wheel_pos, left_quat = self.t.lookupTransform("/base_link", "/front_left_caster", rospy.Time())
        right_wheel_pos, right_quat = self.t.lookupTransform("/base_link", "/front_right_caster", rospy.Time())

        lookahead_global = PoseStamped()
        lookahead_global.header.frame_id = "/map"
        lookahead_global.pose = self.target_pos

        lookahead_local = self.t.transformPose("/base_link", lookahead_global)

        
        print(left_wheel_pos)

        points_arr = []

        p1 = Point32()
        p1.x = left_wheel_pos[0] - self.cart_width / 1.8
        p1.y = left_wheel_pos[1]
        points_arr.append(p1)
        
        p2 = Point32()
        p2.x = right_wheel_pos[0] + self.cart_width / 1.8
        p2.y = right_wheel_pos[1]
        points_arr.append(p2)


        p3 = Point32()
        p3.x = lookahead_local.pose.position.x - self.cart_width / 1.8
        p3.y = lookahead_local.pose.position.y
        points_arr.append(p3)

        p4 = Point32()
        p4.x = lookahead_local.pose.position.x + self.cart_width / 1.8
        p4.y = lookahead_local.pose.position.y
        points_arr.append(p4)

        display_bb.header.frame_id = "/base_link"
        display_bb.header.stamp = rospy.Time(0)
        display_bb.polygon.points = points_arr

        self.boundingbox_pub.publish(display_bb)



        
        



        

        
if __name__ == "__main__":
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        pass

