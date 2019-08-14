#!/usr/bin/env python
#linear step function will probably be replaced with something better
import rospy
import math
from geometry_msgs.msg import Vector3
from navigation_msgs.msg import vel_angle
from navigation_msgs.msg import vel_angle_step
class VelStepNode(object):
   
    def __init__(self):
        self.cmd_msg = vel_angle()
        self.cmd_msg.vel = 0.0
        self.cmd_msg.angle = 0.0
        self.cmd_msg.vel_curr = 0.0 #will eventually be read from some sensor                                    
                                    #topic and published to vel_angle_step by 
                                    #some other node
        self.input_angle = 0.0
        self.input_vel = 0.0
        self.vel_step = 0.0
        self.angle_step = 0.0
        self.control()
        
    def control(self):
        cmd_p = rospy.Publisher('/nav_cmd', vel_angle, queue_size=10)
        instr_s = rospy.Subscriber('/nav_instr', vel_angle_step, self.get_input)
        rospy.init_node('vel_step', anonymous=False)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.cmd_msg.vel = self.step_toward_value(self.cmd_msg.vel, self.input_vel, self.vel_step)
            self.cmd_msg.angle = self.step_toward_value(self.cmd_msg.angle, self.input_angle, self.angle_step)
            cmd_p.publish(self.cmd_msg)
            rospy.loginfo("\n\tForward velocity instruction: "+str(self.cmd_msg.vel)+
                          "\n\tWheel angle instruction:      "+str(self.cmd_msg.angle)+
                          "\n\tCurrent actual velocity:      "+str(self.cmd_msg.vel_curr))
            rate.sleep()
    
    def get_input(self, vel_angle_instr):
        self.vel_curr = vel_angle_instr.vel_curr
        self.input_vel = float(vel_angle_instr.vel)
        self.input_angle = float(vel_angle_instr.angle)
        self.vel_step = float(vel_angle_instr.vel_step)
        self.angle_step = float(vel_angle_instr.angle_step)

    def step_toward_value(self, cur, dest, step): 
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
        VelStepNode()
    except rospy.ROSInterruptException:
        pass
