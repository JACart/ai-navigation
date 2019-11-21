#!/usr/bin/env python 
import curses
import rospy
import sys
import time
import math
from navigation_msgs.msg import VelAngle
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8

class teleop(object):
    def __init__(self):
        rospy.init_node('realtime_param_change')
        self.param_change = rospy.Publisher('/realtime_param_change', Int8, queue_size=10)
        self.a_param_change = rospy.Publisher('/realtime_a_param_change', Int8, queue_size=10)
        #self.vel_sub = rospy.Subscriber('/pose_and_speed', Odometry, self.vel_callback, queue_size = 10)
        self.prev_key = 1
        curses.wrapper(self.get_input)


    def get_input(self, stdscr):
        curses.use_default_colors()
        for i in range(0, curses.COLORS):
            curses.init_pair(i, i, -1)
        w = 119
        a = 97
        s = 115
        d = 100
        x = 120
        y = 121
    
        stdscr.nodelay(True)
        rate = rospy.Rate(10) 
        stdscr.addstr(0,0,'W to increase target speed, s to decrease target speed, a for more left, d for more right')

        change = Int8()
        a_change = 0
        while not rospy.is_shutdown():
            keyval = stdscr.getch()

            if keyval == self.prev_key:
                continue
            if keyval == w:
                change = 1
            elif keyval == s:
                change = -1
            elif keyval == a:
                a_change = -1
            elif keyval == d:
                a_change = 1

            self.prev_key = keyval
            self.a_param_change.publish(a_change)
            self.param_change.publish(change)
            rate.sleep()

if __name__ == "__main__":
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
