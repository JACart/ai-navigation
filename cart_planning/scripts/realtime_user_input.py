#!/usr/bin/env python 
import curses
import rospy
import sys
import time
import math
from navigation_msgs.msg import VelAngle
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, Bool

class teleop(object):
    def __init__(self):
        rospy.init_node('realtime_user_input')
        self.param_change = rospy.Publisher('/realtime_param_change', Int8, queue_size=10)
        self.a_param_change = rospy.Publisher('/realtime_a_param_change', Int8, queue_size=10)
        self.debug_change = rospy.Publisher('/realtime_debug_change', Bool, queue_size=10)
        self.prev_key = 1
        curses.wrapper(self.get_input)
        debug = False


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
        stdscr.addstr(0,0,'W to increase target speed, s to decrease target speed, a for more left, d for more right. Press x to toggle debug display info')

        change = Int8()
        a_change = Int8()
        debug = False
        while not rospy.is_shutdown():
            keyval = stdscr.getch()

            if keyval == self.prev_key:
                continue
            if keyval == w:
                change = .01
            elif keyval == s:
                change = -1
            elif keyval == a:
                a_change = -1
            elif keyval == d:
                a_change = 1
            elif keyval == x:
                if(debug):
                    debug = False
                else:                
                    debug = True
            rospy.loginfo(debug)
            self.prev_key = keyval
            self.a_param_change.publish(a_change)
            self.param_change.publish(change)
            self.debug_change.publish(debug)
            rate.sleep()

if __name__ == "__main__":
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
