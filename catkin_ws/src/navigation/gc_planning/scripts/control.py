#!/usr/bin/env python
#this will be replaced with PID control
import rospy
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from navigation_msgs.msg import vel_angle
from navigation_msgs.msg import vel_angle_step
input_angle = 0.0
input_vel = 0.0
cmd_msg = vel_angle()
cmd_msg.vel = 0.0
cmd_msg.vel = 0.0
vel_step = 0.0
angle_step = 0.0
def control():
    global vel
    global angle
    global vel_step
    global angle_step
    global cmd_msg
    cmd_p = rospy.Publisher('nav_cmd', vel_angle, queue_size=10)
    instr_s = rospy.Subscriber('nav_instr', vel_angle_step, get_input)
    rospy.init_node('control', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	cmd_msg.vel = step_toward_value(cmd_msg.vel, input_vel, vel_step)
        cmd_msg.angle = step_toward_value(cmd_msg.angle, input_angle, angle_step)
        cmd_p.publish(cmd_msg)
	rospy.loginfo("\n\tForward velocity instruction: "+str(float(cmd_msg.vel))+
		      "\n\tWheel angle instruction:      "+str(float(cmd_msg.angle))+"\n")
        rate.sleep()

def get_input(vel_angle_instr):
    global input_vel
    global input_angle
    global angle_step
    global vel_step
    input_vel = float(vel_angle_instr.vel)
    input_angle = float(vel_angle_instr.angle)
    vel_step = float(vel_angle_instr.vel_step)
    angle_step = float(vel_angle_instr.angle_step)

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
