#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

MAX_SPEED = 100
#equal to 10 mph 4.4
MAX_ANGLE = 45
ackermann_p = None
def converter():
    global ackermann_p
    steering_angle_velocity = 0.0
    acceleration = 0.0
    jerk = 0.0
    rospy.init_node('cmd_vel_to_ackermann_msgs', anonymous=False)
    cmd_vel_s = rospy.Subscriber('cmd_vel', Twist, convert)
    ackermann_p = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	rate.sleep()
def convert(twist_msg):
    global ackermann_p
    ack = AckermannDrive()
    ack.speed = calc_speed(twist_msg.linear.x)
    ack.steering_angle = calc_angle(twist_msg.angular.z)
    ackermann_p.publish(ack)
def calc_speed(twist_vel):
    global MAX_SPEED
    if(twist_vel > MAX_SPEED):
        return MAX_SPEED
    elif(twist_vel < -MAX_SPEED):
        return -MAX_SPEED
    else:
        return twist_vel
def calc_angle(twist_angle):	
    global MAX_ANGLE
    if(twist_angle > MAX_ANGLE):
        return math.radians(MAX_ANGLE)
    elif(twist_angle < -MAX_ANGLE):
        return math.radians(-MAX_ANGLE)
    else:
        return math.radians(twist_angle)

if __name__ == '__main__':
    try:
        converter()
    except rospy.ROSInterruptException:
        pass
