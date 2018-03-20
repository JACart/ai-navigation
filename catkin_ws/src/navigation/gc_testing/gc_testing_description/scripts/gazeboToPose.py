#!/usr/bin/env python

#This node is for the simulation
#It takes the odometry from the simulated car and posts it to
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

def gps():

    rospy.init_node('gps', anonymous=False)
    odom_s = rospy.Subscriber('odom', Odometry, read_gps)
    odom_p = rospy.Publisher('pose_and_speed', Odometry, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	    rate.sleep()

def read_gps(odom_msg):
    p = Point()
    p.x = odom_msg.pose.pose.position.x
    p.y = odom_msg.pose.pose.position.y
    p.z = odom_msg.pose.pose.position.z
    #print "x:", odom_msg.pose.pose.position.x, " y:", odom_msg.pose.pose.position.x
    print odom_msg.header.frame_id
    odom
    #TODO PUBLISH HERE

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
