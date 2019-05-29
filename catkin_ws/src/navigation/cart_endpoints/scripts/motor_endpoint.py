#!/usr/bin/env python

import serial
import rospy
from navigation_msgs.msg import VelAngle
from navigation_msgs.msg import EmergencyStop
SPEED_PORT = '/dev/ttyACM0' #hardcoded depending on computer
TURN_PORT = ''

class MotorEndpoint(object):

    def __init__(self):
        global SPEED_PORT
        self.killswitch = False
        self.current_speed = 0.0
        self.goal_speed = 0.0
        self.goal_angle = 0.0

        self.cmd_msg = None
        """ Set up the node. """
        rospy.init_node('motor_endpoint')
        rospy.loginfo("Starting motor node!")
        #Connect to arduino for sending speed
        try:
            self.speed_ser = serial.Serial(SPEED_PORT, 19200, write_timeout=0)
        except Exception as e:
            print "Motor_endpoint: " + str(e)
            rospy.logerr("Motor_endpoint: " + str(e))
            exit(0)

        rospy.logerr("Speed serial established")
        """
        Connect to arduino for steering
        """
        self.speed_string = ''
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback,
                                                  queue_size=10)
        self.killswitch_subscriber = rospy.Subscriber('/emergency_stop', EmergencyStop,
                                                      self.kill_callback, queue_size=10)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.cmd_msg is not None:
                self.send_to_motors()
            rate.sleep()

    def motion_callback(self, planned_vel_angle):

        if self.killswitch:
            self.speed_ser.write(":0.0,0.0,0.0".encode())
            rospy.loginfo("Killswitch activated")

        self.cmd_msg = planned_vel_angle


    def kill_callback(self, data):
        self.killswitch = data.emergency_stop


    def send_to_motors(self):
        spd = self.cmd_msg.vel

        wheel_ang = self.cmd_msg.angle
        if wheel_ang > 28:
            wheel_ang = 28
        if wheel_ang < -42:
            wheel_ang = -42

        angle = wheel_ang * -12
        cur_spd = self.cmd_msg.vel_curr



        if spd > 0:
            spd = 2.0
            cur_spd = 0.01
        else:
            spd = 0.0

        angle = (int)(angle)

        print "speed: " + str(spd) + " angle: " + str(angle) + " cur_spd: " + str(cur_spd)

        msg_to_motors = ':' + str(spd) + ',' + str(cur_spd) + "," + str(angle)
        self.speed_ser.write(msg_to_motors.encode())

if __name__ == "__main__":
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass
