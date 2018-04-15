#!/usr/bin/env python 
import rospy
import serial
from navigation_msgs.msg import VelAngle
from navigation_msgs.msg import EmergencyStop
speed_port = '/dev/ttyACM0' #hardcoded depending on computer
turn_port = ''

class MotorEndpoint(object):
    
    def __init__(self):
        global speed_port
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
            self.speed_ser = serial.Serial(speed_port, 9600)
        except:
            print("Error connecting to serial port")
            rospy.logerr("Motor_endpoint: Error connecting to serial port")
            exit(0)
            
        rospy.loginfo("Speed serial established")
        """
        #Connect to arduino for steering
        turn_ser = serial.Serial(turn_port, 9600)
        rospy.loginfo("Steering serial established")
        """
        self.speed_string = ''
        response = "No response yet"
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', VelAngle, self.motion_callback, queue_size = 10)
        self.killswitch_subscriber = rospy.Subscriber('/emergency_stop', EmergencyStop, self.kill_callback, queue_size = 10)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.cmd_msg is not None:
                self.send_to_motors()
            rate.sleep()
        
    def motion_callback(self, planned_vel_angle):

        if self.killswitch:
            self.speed_ser.write(":0.0,0.0,0.0".encode())
            rospy.loginfo("Killswitch activated")
        '''else:
            self.goal_speed = planned_vel_angle.vel
            self.goal_angle = planned_vel_angle.angle
            self.current_speed = planned_vel_angle.vel_curr
            self.speed_string =  ':'+str(self.goal_speed)+ ',' + str(self.current_speed)+','+str(self.goal_angle)
            self.speed_ser.write(self.speed_string.encode())
            #response = self.speed_ser.readline(eol:="\n")
            rospy.loginfo("String being sent: "+self.speed_string)
            #rospy.loginfo("Response: "+response)'''

        self.cmd_msg = planned_vel_angle

        
    def kill_callback(self, data):
        self.killswitch = data.emergency_stop


    def send_to_motors(self):
        spd = self.cmd_msg.vel
        angle = self.cmd_msg.angle
        cur_spd = self.cmd_msg.vel_curr
        print ("speed: " + str(spd) + " angle: " + str(angle) + "cur_spd: " + str(cur_spd))  
        msg_to_motors =  ':'+str(spd)+','+str(cur_spd)+","+ str(angle)
        self.speed_ser.write(msg_to_motors.encode())
        
        
if __name__ == "__main__": 
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass
