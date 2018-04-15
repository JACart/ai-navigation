#!/usr/bin/env python 
import rospy
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
	    self.twist_msg.linear.x = (vel_angle.angle/180)*math.pi
	    self.twist_msg.linear.y = vel_angle.vel
	    self.cmd_p.publish(self.twist_msg)
	
if __name__ == "__main__":
    VelAngleToTwist()
