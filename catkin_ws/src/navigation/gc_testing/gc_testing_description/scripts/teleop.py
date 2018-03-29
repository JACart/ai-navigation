#!/usr/bin/env python 

import rospy
import readchar
from navigation_msgs.msg import vel_angle

class teleop(object):
    def __init__(self):
        rospy.init_node('teleop')

        self.motion_pub = rospy.Publisher('/nav_cmd', vel_angle, queue_size=10)

        print 'Move with WSAD\nCTRL-C to exit\n'


        while (True):
            key = readchar.readkey()

            if (ord(key) == 3):
                exit(0)

            msg = vel_angle()
            msg.vel_curr = ord(key)
            msg.vel = 0
            msg.angle = 0

            #W pressed
            if (ord(key) == 119):
                msg.vel = 4
            #A pressed
            elif (ord(key) == 97):
                msg.vel = 1
                msg.angle = -360
            #S pressed
            elif (ord(key) == 115):
                msg.vel = 0.01
            #D pressed
            elif (ord(key) == 100):
                msg.vel = 1
                msg.angle = 360

            self.motion_pub.publish(msg)

        rospy.spin()

if __name__ == "__main__":
    try:
	teleop()
    except rospy.ROSInterruptException:
	pass
