#!/usr/bin/env python

import socket
import rospy
from navigation_msgs.msg import EmergencyStop, VehicleState
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import NDTStat

class CartHealth(object):

    def __init__(self):
        rospy.init_node('cart_health_monitor')

        self.is_navigating = False

        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)

        self.vehicle_state_sub = rospy.Subscriber('/vehicle_state', VehicleState, self.status_update)
        self.ndt_stat_sub = rospy.Subscriber('/ndt_stat', NDTStat, self.ndt_stat_cb)
	
        self.vehicle_speed_sub = rospy.Subscriber('/estimate_twist', TwistStamped, self.speed_check)
        rospy.spin()

    # Keep track if cart is navigating
    def status_update(self, msg):
        self.is_navigating = False
        if msg.is_navigating:
            self.is_navigating = True

    # Check the localizer's current health
    def ndt_stat_cb(self, msg):
        cur_fitness = msg.score
        if cur_fitness > 1.0 and cur_fitness < 1.5:
            rospy.logwarn("Localizer has bad health, current health: " + str(cur_fitness))
        elif cur_fitness >= 1.5:
            rospy.logfatal("Localizer has extremely bad health")
            if self.is_navigating:
                rospy.logfatal("Sending Emergency Stop due to bad localization!")
                self.send_stop()
                self.is_navigating = False

    def speed_check(self, msg):
        if msg.twist.linear.x >= 3:
            rospy.logfatal("Overspeeding! Sending Emergency Stop message!")
            self.send_stop()

    def send_stop(self):
        stop_msg = EmergencyStop()
            stop_msg.emergency_stop = True
            self.emergency_stop_pub.publish(stop_msg)

if __name__ == "__main__":
    try:
        CartHealth()
    except rospy.ROSInterruptException:
        pass

