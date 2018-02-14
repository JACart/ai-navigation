#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
input_angle = 0.0
input_vel = 0.0
vel = 0.0
angle = 0.0
vel_step = 0.0
angle_step = 0.0
def control():
    global vel
    global angle
    global vel_step
    global angle_step
    cmd_vel_p = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    instr_s = rospy.Subscriber('plan_instr', String, get_input)
    rospy.init_node('control', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	vel = step_toward_value(vel, input_vel, vel_step)
        angle = step_toward_value(angle, input_angle, angle_step)
        twist_msg = Twist()
	twist_msg.linear.x = vel
	twist_msg.angular.z = angle
        cmd_vel_p.publish(twist_msg)
	rospy.loginfo("\n\tForward velocity instruction: "+str(float(vel))+
		      "\n\tWheel angle instruction:      "+str(float(angle))+"\n")
        rate.sleep()

def get_input(value):
    global input_vel
    global input_angle
    global angle_step
    global vel_step
    arr = value.data.split(' ')
    input_vel = float(arr[0]) if arr.__len__() > 0 else 0.0
    input_angle = float(arr[1]) if arr.__len__() > 1 else 0.0
    vel_step = float(arr[2]) if arr.__len__() > 2 else 0.01
    angle_step = float(arr[3]) if arr.__len__() > 3 else 10.0

def step_toward_value(cur, dest, step): 
    diff = abs(dest - cur)
    if(diff < step):
	step = diff
    if cur > dest: 
	return cur-step
    elif cur < dest: 
	return cur+step
    else:
	return cur

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
