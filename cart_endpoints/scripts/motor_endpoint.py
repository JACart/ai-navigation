#!/usr/bin/env python

import serial
import rospy
import bitstruct
from navigation_msgs.msg import VelAngle
from navigation_msgs.msg import EmergencyStop
from std_msgs.msg import Int8, Bool
import time
cart_port = '/dev/ttyUSB9' #hardcoded depending on computer

# STATES:
MOVING = 0 
BRAKING = 1
STOPPED = 2

class MotorEndpoint(object):

    def __init__(self):
        global cart_port
        self.current_speed = 0.0
        self.new_vel = True
        self.debug = False
        self.stop = False
        self.gentle_stop = False
        self.delay_print = 0
        self.brake = int(0)
        self.drove_since_braking = True
        self.cmd_msg = None
        # Time (seconds) to ramp up to full brakes
        self.brake_time = 3
        self.node_rate = 10
        self.state = STOPPED
        self.stopping_time = 0
        self.step_size = 255.0/(self.node_rate*self.brake_time)
        """ Set up the node. """
        rospy.init_node('motor_endpoint')
        rospy.loginfo("Starting motor node!")
        #Connect to arduino for sending speed
        try:
            self.speed_ser = serial.Serial(cart_port, 57600, write_timeout=0)
        except Exception as e:
            print( "Motor_endpoint: " + str(e))
            rospy.logerr("Motor_endpoint: " + str(e))

        rospy.loginfo("Speed serial established")
        """
        Connect to arduino for steering
        """
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback,
                                                  queue_size=10)
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
        
        rate = rospy.Rate(self.node_rate)

        while not rospy.is_shutdown():
            if self.cmd_msg is not None:
                self.endpoint_calc()
            rate.sleep()

    def motion_callback(self, planned_vel_angle):
        self.cmd_msg = planned_vel_angle

        if self.cmd_msg.vel > 0 and (self.state == STOPPED or self.state == BRAKING) and (time.time()- self.stopping_time) > 10:
            self.state = MOVING
            self.brake = 0 # make sure we take the foot off the brake

        elif self.state == MOVING and self.cmd_msg.vel <= 0:  # Brakes are hit
            self.state = BRAKING
            self.brake = 0 # ramp up braking from 0
            self.stopping_time = time.time()
            
        self.new_vel = True     

    def stop_callback(self, msg):
        self.stop = msg.data

    def gentle_callback(self, msg):
        self.gentle_stop = msg.data
        self.stop_done = False

    def debug_callback(self, msg):
        self.debug = msg.data


    def endpoint_calc(self):
        #The first time we get a new target speed and angle we must convert it
        
        if self.new_vel:
            self.new_vel = False
            self.cmd_msg.vel *= 50       # Rough conversion from m/s to cart controller units
            self.cmd_msg.vel_curr *= 50  # Rough conversion from m/s to cart controller units
            if self.cmd_msg.vel > 254:
                self.cmd_msg.vel = 254
            if self.cmd_msg.vel < -254:
                self.cmd_msg.vel = -254
            if self.cmd_msg.vel_curr > 254:
                self.cmd_msg.vel_curr = 254
            if self.cmd_msg.vel < 0:
                rospy.logwarn("NEGATIVE VELOCITY REQUESTED FOR THE MOTOR ENDPOINT!")
        target_speed = int(self.cmd_msg.vel) #float64
        current_speed = int(self.cmd_msg.vel_curr) #float64
        #adjust the target_angle range from (-45 <-> 45) to (0 <-> 100)
        # rospy.loginfo("Angle before adjustment: " + str(self.cmd_msg.angle))
        if(self.cmd_msg.angle < -45):
            self.cmd_msg.angle = -45
        if(self.cmd_msg.angle > 45):
            self.cmd_msg.angle = 45
        target_angle = 100 - int(( (self.cmd_msg.angle + 45) / 90 ) * 100)
        
            
        #if debug printing is requested print speed and angle info
        if self.debug:
            self.delay_print -= 1
            if self.delay_print <= 0:
                self.delay_print = 5
                rospy.loginfo("Endpoint Angle: " + str(target_angle))
                rospy.loginfo("Endpoint Speed: " + str(target_speed))

        if self.state == STOPPED:
            self.brake = 0
            target_speed = 0
        elif self.state == BRAKING:
            self.brake = float(min(255, self.brake + self.step_size))
            if self.brake >= 255:  # We have reached maximum braking!
                self.state = STOPPED
            target_speed = 0

        self.pack_send(target_speed, int(self.brake), target_angle)
    
    def pack_send(self, throttle, brake, steer_angle):
        data = bytearray(b'\x00' * 5)
        bitstruct.pack_into('u8u8u8u8u8', data, 0, 42, 21, abs(throttle), brake, steer_angle)
        self.speed_ser.write(data)

if __name__ == "__main__":
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass
