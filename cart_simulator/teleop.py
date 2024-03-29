#!/usr/bin/env python 
import curses
import rospy
import sys
import time
import math
from navigation_msgs.msg import VelAngle
from nav_msgs.msg import Odometry

class teleop(object):
    def __init__(self):
        rospy.init_node('teleop')
        self.msg = VelAngle()
        self.motion_pub = rospy.Publisher('/nav_cmd', VelAngle, queue_size=10)
        #self.vel_sub = rospy.Subscriber('/pose_and_speed', Odometry, self.vel_callback, queue_size = 10)

        self.prev_key = 1
        self.cur_vel = 0.0

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
        p = 112
        o = 111
        i = 105
        u = 117
        l = 108
    
        angle_max = 70
        angle_step = 0.5
        velstr = 'Hard stop     '
        anglestr = 'Center wheels '
        stdscr.nodelay(True)
        rate = rospy.Rate(10) 
        stdscr.addstr(0,0,'Move with WASD, X for hard stop and Y for centering the wheel')
        stdscr.addstr(1,0,'p= comfortable stop, o= obstacle 2m, i= obstacle 5m')
        stdscr.addstr(2,0,'u= obstacle in 10m, l = start at 2.7s')
        stdscr.addstr(3,0,'CTRL-C to exit')
        stdscr.addstr(4,0,'TURNING       WHEEL ANGLE')
        stdscr.addstr(6,0,'FORWARD MOVEMENT')

        while not rospy.is_shutdown():
            keyval = stdscr.getch()

            if keyval == self.prev_key:
                continue
            if keyval == w:
                self.msg.vel_curr = 0.01
                self.msg.vel = 2.0
                velstr = "Full throttle "
            elif keyval == a:
                if self.msg.angle + angle_step <= angle_max:
                    self.msg.angle += angle_step
                    anglestr = "Turn left     "
            elif keyval == s:
                self.msg.vel_curr = 0.4
                self.msg.vel = 0.01
                velstr = "No throttle   "
            elif keyval == d:
                if self.msg.angle - angle_step >= -angle_max:
                    self.msg.angle -= angle_step
                    anglestr = "Turn right    "
            elif keyval == x:
                self.msg.vel = -1.0
                velstr = "Hard stop     "
            elif keyval == y:
                self.msg.angle = 0.0
                anglestr = "Center wheel  "
            elif keyval == p:
                self.msg.vel_curr = 2.7
                self.msg.vel = 0
                velstr = "comfortable stop          "
            elif keyval == o:
                self.msg.vel_curr = 2.7
                self.msg.vel = -2
                velstr = "obstacle in 2m - stop              "
            elif keyval == i:
                self.msg.vel_curr = 2.7
                self.msg.vel = -5
                velstr = "obstacle in 5m - stop              "
            elif keyval == u:
                self.msg.vel_curr = 2.7
                self.msg.vel = -10
                velstr = "obstacle in 10m - stop              "
            elif keyval == l:
                self.msg.vel_curr = 0.01
                self.msg.vel = 2.7
                velstr = "start moving at 2.7 m/s"

            self.prev_key = keyval
            self.motion_pub.publish(self.msg)
            outstr = anglestr +  velstr + str(self.msg.angle)
            stdscr.addstr(5,0,anglestr + str(self.msg.angle))
            stdscr.addstr(7,0,velstr)
            rate.sleep()

if __name__ == "__main__":
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass