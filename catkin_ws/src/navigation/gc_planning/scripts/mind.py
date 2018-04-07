#!/usr/bin/env python 
import gps_util
import rospy
from navigation_msgs.msg import WaypointsArray, LatLongPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped


import cubic_spline_planner #might want to move where this is
import pure_pursuit #same as above
import matplotlib.pyplot as plt #THIS IS TEMPORARY

class mind(object):
    def __init__(self):
        rospy.init_node('mind')

        self.waypoints_s = rospy.Subscriber('/waypoints', WaypointsArray, self.waypoints_callback, queue_size=10) 
        self.xyz_waypoint_pub = rospy.Publisher('/xyz_waypoints', PointStamped, queue_size=10, latch = True)

        rospy.spin()


    #This reads from the waypoints topic and TODO
    def waypoints_callback(self, msg):

        google_points = []

        #Reads each point in the waypoint topic
        for gps_point in msg.waypoints:
            point = gps_util.get_point(gps_point)
            google_points.append(point)

            #Publishes to rviz for visualization
            #self.xyz_waypoint_pub.publish(self.add_header(point))

        print len(google_points)

        #just for testing
        x = []
        y = []

        for p in google_points:
            x.append(p.x)
            y.append(p.y)

        #plt.scatter(x,y)
        #TODO end testing here

        li = gps_util.add_intermediate_points(google_points, 15.0)
        print len(li)

        ax = []
        ay = []

        for p in li:
            ax.append(p.x)
            ay.append(p.y)

        #just for testing
        '''f2 = plt.figure()
        plt.scatter(ax, ay)'''

        #TODO end testing here



        #calculate the spline
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)

        #just for testing
        '''f3 = plt.figure()
        plt.plot(ax, ay, "xb", label="input")
        plt.plot(cx, cy, "-r", label="spline")
        plt.plot(x, y, "-g", label="tracking")
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.legend()'''

        #plt.ion()
        #plt.show()
        #TODO end testing here


        #================================================ pure persuit copy/pase ===============================================

        k = 0.1  # look forward gain
        Lfc = 1.0  # look-ahead distance
        Kp = 1.0  # speed propotional gain
        dt = 0.1  # [s]
        L = 2.9  # [m] wheel base of vehicle

        target_speed = 10.0 / 3.6  # [m/s]
        T = 100.0  # max simulation time

        # initial state
        state = State(x=50.0, y=-80.0, yaw=0.0, v=0.0)

        lastIndex = len(cx) - 1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        target_ind = pure_pursuit.calc_target_index(state, cx, cy)

        while T >= time and lastIndex > target_ind:
            ai = pure_pursuit.PIDControl(target_speed, state.v)
            di, target_ind = pure_pursuit.pure_pursuit_control(state, cx, cy, target_ind)
            state = pure_pursuit.update(state, ai, di)

            time = time + dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

            #if show_animation:
            #f4 = plt.figure()

            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

        # Test
        assert lastIndex >= target_ind, "Cannot goal"

        #if show_animation:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        flg, ax = plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


        #================================================ pure persuit copy/pase END ===============================================


    def add_header(self, point):
        stamped = PointStamped()
        stamped.header = Header()
        stamped.header.frame_id = '/odom'
        stamped.point = point
        return stamped


if __name__ == "__main__":
    try:
	mind()
    except rospy.ROSInterruptException:
	pass
