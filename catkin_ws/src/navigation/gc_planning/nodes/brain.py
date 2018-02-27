#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from navigation_msgs.msg import emergency_stop, lat_long_point 
from ..scripts import control, point_to_goal
class Brain(object):

    def __init__(self):
        self.vel_curr = 0.0
	#publishers                                    
        self.cmd_vel_p = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #subscribers	
	self.gps_s = rospy.Subscriber('/gps', Odometry, self.get_input, self.gps_callback, queue_size=10)
	self.goal_s = rospy.Subscriber('/goal', lat_long_point, self.goal_callback, queue_size=10) 
	self.killswitch_s = rospy.Subscriber('/stop', emergency_stop, self.killswitch_callback, queue_size=10) 
	self.point_cloud_s = rospy.Subscriber('/point_cloud', PointCloud2, self.point_cloud_callback, queue_size=10) 
	control() 
 
    def control(self):
        rospy.init_node('Brain', anonymous=False)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
       	    #if killswitch publish 0 to cmd_vel
            rate.sleep()    
    
    def gps_callback(self, msg):
        pass
	
    def goal_callback(self, msg):
	pass

    def killswitch_callback(self, msg):
	pass

    def point_cloud_callback(self, msg):
	pass
if __name__ == '__main__':
    try:
       Brain()
    except rospy.ROSInterruptException:
        pass
