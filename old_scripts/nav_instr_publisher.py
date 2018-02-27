#!/usr/bin/env python
import rospy
from navigation_msgs.msg import vel_angle_step

class NavInstrPublisher(object):
    
    def __init__(self):
        """ Set up the node. """
        self.instr_pub = rospy.Publisher('/nav_instr', vel_angle_step, queue_size = 10)
        rospy.init_node('nav_pub')
        self.vel_msg = vel_angle_step()
        self.vel_msg.angle_step = 5
        self.vel_msg.vel_step = 0.3
        self.prompt()
        #rospy.sleep(10)   
        
    def prompt(self):  
        rate = rospy.Rate(10)
        while True:# not rospy.is_shutdown():
            self.vel_msg.vel_curr = float(input("Input current velocity:"))
            self.vel_msg.vel= float(input("Input desired velocity:"))
            self.vel_msg.angle = float(input("Input desired steering angle:"))
            self.instr_pub.publish(self.vel_msg)
            rospy.loginfo("Published data to nav_instr:\n" + self.)
            rate.sleep()
        
if __name__ == "__main__": 
    try:
        NavInstrPublisher()
    except rospy.ROSInterruptException:
        pass
        