#!/usr/bin/env python

import socket
import rospy
from navigation_msgs.msg import VehicleState, EmergencyStop
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import NDTStat
from std_msgs.msg import Bool, Float32

class CartHealth(object):

    def __init__(self):
        rospy.init_node('cart_health_monitor')

        self.vehicle_state = VehicleState()
        self.stopping = False
        # Safety maximums (Speed, and localizer health)
        self.max_speed = rospy.get_param('max_speed')
        self.max_ndt_health = rospy.get_param('max_ndt_health')

        self.stop_pub = rospy.Publisher('/stop', EmergencyStop, queue_size=10)

        self.vehicle_state_sub = rospy.Subscriber('/vehicle_state', VehicleState, self.status_update)
        self.ndt_stat_sub = rospy.Subscriber('/ndt_stat', NDTStat, self.ndt_stat_cb)
	
        self.vehicle_speed_sub = rospy.Subscriber('/estimated_vel_mps', Float32, self.speed_check)
        rospy.spin()

    # Keep track if cart is navigating
    def status_update(self, msg):
        self.vehicle_state = msg

    # Check the localizer's current health
    def ndt_stat_cb(self, msg):
        """ Makes sure there is a decent localization at all times during cart navigation.
        Once stopped for this reason there is no allowed resuming, a system restart is required in this case.

        Args:
            msg(NDTStat ROS MSG): Message coming from the NDT Localizer containing health
        """
        if self.vehicle_state.is_navigating:
            cur_fitness = msg.score
            # If we have a decent amount of bad health but not as bad as maximum
            if cur_fitness > 2.0 and cur_fitness < (self.max_ndt_health-1):
                rospy.logwarn("Localizer has bad health, current health: " + str(cur_fitness))
            elif cur_fitness >= self.max_ndt_health:
                rospy.logfatal("Localizer has extremely bad health")
                rospy.logfatal("Sending Emergency Stop due to bad localization!")
                self.send_stop(True, "Localizer")

    def speed_check(self, msg):
        """ Makes sure the cart doesn't overspeed, if for some reason the cart overspeeds stop it

        Args:
            msg(Float32 ROS MSG): Contains estimated speed of cart in Meters per Second
        """
        if msg.data >= self.max_speed:
            rospy.logfatal("Overspeeding! Sending Emergency Stop message!")
            self.send_stop(True, "Overspeed")
            self.stopping = True
        elif self.stopping == True and msg.data <= 1:
            self.stopping = False
            rospy.loginfo("Speed has been reduced, resuming ride!")
            self.send_stop(False, "Overspeed")

    def send_stop(self, stop, sender_id):
        """ Function for quickly defining and sending an emergency stop message

        Args:
            stop(Boolean): Make a request to stop(True) or continue(False)
            sender_id(String): Unique identifier of function or node making the request 
        """
        stop_msg = EmergencyStop()
        stop_msg.emergency_stop = stop
        stop_msg.sender_id.data = sender_id
        self.stop_pub.publish(stop_msg)

if __name__ == "__main__":
    try:
        CartHealth()
    except rospy.ROSInterruptException:
        pass

