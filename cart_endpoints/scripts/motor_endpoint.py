#!/usr/bin/env python

import serial
import rospy
import bitstruct
from navigation_msgs.msg import VelAngle
from navigation_msgs.msg import EmergencyStop
from std_msgs.msg import Int8, Bool
cart_port = '/dev/ttyUSB0' #hardcoded depending on computer

class MotorEndpoint(object):

    def __init__(self):
        global cart_port
        self.killswitch = False
        self.current_speed = 0.0
        self.goal_speed = 0.0
        self.goal_angle = 0.0
        self.new_vel = True
        self.debug = False
        self.angle_adjust = 0
        self.stopping = False
        self.delay_print = 0

        self.cmd_msg = None
        """ Set up the node. """
        rospy.init_node('motor_endpoint')
        rospy.loginfo("Starting motor node!")
        #Connect to arduino for sending speed
        try:
            rospy.loginfo("remove comments")
            #self.speed_ser = serial.Serial(cart_port, 57600, write_timeout=0)
        except Exception as e:
            print( "Motor_endpoint: " + str(e))
            rospy.logerr("Motor_endpoint: " + str(e))
            #exit(0)

        rospy.loginfo("Speed serial established")
        """
        Connect to arduino for steering
        """
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback,
                                                  queue_size=10)
        self.stop_subscriber = rospy.Subscriber('/emergency_stop', Bool,
                                                      self.stop_callback, queue_size=10)
        self.param_subscriber = rospy.Subscriber('/realtime_a_param_change', Int8, self.param_callback, queue_size=10)
        
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
        
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.cmd_msg is not None:
                self.send_to_motors()
            rate.sleep()

    def motion_callback(self, planned_vel_angle):
        self.cmd_msg = planned_vel_angle  
        self.new_vel = True     

    def param_callback(self, msg):
        self.angle_adjust += (msg.data * 10)

    def stop_callback(self, msg):
        print("Motor endpoint: " + str(msg.data))
        self.stopping = msg.data

    def debug_callback(self, msg):
        self.debug = msg.data


    def send_to_motors(self):
        if self.new_vel:
            #self.new_vel = False #this is set to false after adjusting the new target angle below
            self.cmd_msg.vel *= 50
            self.cmd_msg.vel_curr *= 50
        if self.cmd_msg.vel > 254:
            self.cmd_msg.vel = 254
        if self.cmd_msg.vel < -254:
            self.cmd_msg.vel = -254
        if self.cmd_msg.vel_curr > 254:
            self.cmd_msg.vel_curr = 254
        target_speed = int(self.cmd_msg.vel) #float64
        current_speed = int(self.cmd_msg.vel_curr) #float64

        target_angle = 100 - int(( (self.cmd_msg.angle + 70) / 140 ) * 100)
        if self.new_vel:
            self.new_vel = False
            if target_angle < self.angle_adjust:
                target_angle -= (10 + int(self.angle_adjust/2))
            if target_angle > 100 - self.angle_adjust:
                target_angle += (10 + int(self.angle_adjust/2))
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 5
                rospy.loginfo("Endpoint Angle: " + str(target_angle))
                rospy.loginfo("Endpoint Speed: " + str(target_speed))
        data = (target_speed,current_speed,target_angle)
        data = bytearray(b'\x00' * 5)

        if self.stopping:
            print("STOPPING")
            bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, 0, 200, 50) #currently a flat 200 braking number
        else:
            if target_speed < 0:
                bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, 0, abs(target_speed), target_angle)
            else:
                bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, abs(target_speed), 0, target_angle)

        #self.speed_ser.write(data) 


if __name__ == "__main__":
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass
