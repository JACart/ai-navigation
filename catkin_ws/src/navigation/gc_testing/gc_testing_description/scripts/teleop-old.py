#!/usr/bin/env python 

import rospy
import readchar
import time
import math
from navigation_msgs.msg import VelAngle
from nav_msgs.msg import Odometry

class teleop(object):
    def __init__(self):
        rospy.init_node('teleop')

        self.msg = VelAngle()
        self.motion_pub = rospy.Publisher('/nav_cmd', VelAngle, queue_size=10)
        self.vel_sub = rospy.Subscriber('/pose_and_speed', Odometry, self.vel_callback, queue_size = 10)

        print 'Move with WSAD\nCTRL-C to exit\n'
        rate = rospy.Rate(10)

        self.prev_key = 1
        self.cur_vel = 0.0

        while (True):
            key = readchar.readkey()

            if (ord(key) == 3):
                exit(0)


            #msg.vel_curr = 0 #ord(key)
            #msg.vel = 0.0
            #msg.angle = 0.0

            #W pressed
            if (ord(key) == 119):
                self.msg.vel_curr = 0.01 #self.key_check(ord(key), 4.0)
                self.msg.vel = .4
            #A pressed
            elif (ord(key) == 97):
                self.msg.angle = 720
            #S pressed
            elif (ord(key) == 115):
                self.msg.vel_curr = 0.4 #self.key_check(ord(key), 0.01)
                self.msg.vel = 0.01
            #D pressed
            elif (ord(key) == 100):
                self.msg.angle = -720
	    #X pressed -- hard stop
            elif (ord(key) == 120):
		self.msg.vel = 0.0
	    #Y pressed -- stop stearing
	    elif (ord(key) == 121):
		self.msg.angle = 0.0
            self.motion_pub.publish(self.msg)
            #rate.sleep()

            #time.sleep(0.05)

        #rospy.spin()

    def key_check(self, key, target):
        #print key, self.prev_key
        if key != self.prev_key:
            self.prev_key = key
            return self.cur_vel

        if self.cur_vel < target - 0.05:
            self.cur_vel = self.cur_vel + 0.05
            return self.cur_vel

        if self.cur_vel > target + 0.05:
            self.cur_vel = self.cur_vel - 0.05
            return self.cur_vel

        self.cur_vel = target
        return self.cur_vel

    def vel_callback(self, msg):
        x_spd = msg.twist.twist.linear.x
        y_spd = msg.twist.twist.linear.y
        self.cur_vel = math.sqrt(x_spd ** 2 + y_spd ** 2)


if __name__ == "__main__":
    try:
	teleop()
    except rospy.ROSInterruptException:
	pass
