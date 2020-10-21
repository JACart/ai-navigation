#!/usr/bin/env python

import serial
import rospy
import bitstruct
from navigation_msgs.msg import VelAngle
from navigation_msgs.msg import EmergencyStop
from std_msgs.msg import Int8, Bool
cart_port = '/dev/ttyUSB9' #hardcoded depending on computer

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
        self.node_rate = 5
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
        self.stop_subscriber = rospy.Subscriber('/stop_vehicle', Bool,
                                                      self.stop_callback, queue_size=10)
        self.gentle_stop_subscriber = rospy.Subscriber('/gentle_stop', Bool, self.gentle_callback, queue_size=10)
        
        self.debug_subscriber = rospy.Subscriber('/realtime_debug_change', Bool, self.debug_callback, queue_size=10)
        
        rate = rospy.Rate(self.node_rate)

        while not rospy.is_shutdown():
            if self.cmd_msg is not None:
                self.endpoint_calc()
            rate.sleep()

    def motion_callback(self, planned_vel_angle):
        self.cmd_msg = planned_vel_angle  
        self.new_vel = True     

    def stop_callback(self, msg):
        self.stop = msg.data

    def gentle_callback(self, msg):
        self.gentle_stop = msg.data

    def debug_callback(self, msg):
        self.debug = msg.data


    def endpoint_calc(self):
        #The first time we get a new target speed and angle we must convert it
        
        if self.new_vel:
            self.new_vel = False
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

        # If an emergency stop
        if self.stop:
            # Pass in max 255 for brakes
            self.brake = 255
            target_speed = 0
        elif self.gentle_stop:
            # This can later be adjusted to have dynamic braking times
            step_size = 255/(self.node_rate*self.brake_time)
            self.brake += step_size
            self.brake = int(min(255, self.brake))
            self.pack_send(0, self.brake, target_angle)
            target_speed = 0
        else:
            self.brake = 0


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
