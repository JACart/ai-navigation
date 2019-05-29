#!/usr/bin/env python 
import rospy
import math
from navigation_msgs.msg import VelAngle
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
class VelAngleToTwist(object):
    def __init__(self):
        rospy.init_node('VelAngleToTwist')
        self.cmd_p = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.vel_angle_s = rospy.Subscriber('/nav_cmd', VelAngle, self.convert, queue_size = 10)
        self.twist_msg = Twist()
        rospy.spin()
    def convert(self, vel_angle):
        #https://arxiv.org/pdf/1604.07446.pdf used equation III.8


        vel_r = vel_angle.vel

        delta = (vel_angle.angle/180)*math.pi #in radians
        vel_f = vel_r / math.cos(delta)
        l = 2.337 #length of car
        theta_dot = (vel_f / l) * math.sin(delta)



        self.twist_msg.angular.z= theta_dot
        self.twist_msg.linear.x = vel_angle.vel




        self.cmd_p.publish(self.twist_msg)
if __name__ == "__main__":
    VelAngleToTwist()
