#!/usr/bin/env python
#this linear step function will probably be replaced with something better
from datetime import datetime
import math

vel_step = 0.05 #per timestep
angle_step = 1.0 #per timestep

class Control(object):
    def __init__(self, timestephz):
	global vel_step
	global angle_step
	self.timestep = float(timestephz)/1000 #convert hz to millis
        self.vel_step = vel_step
        self.angle_step = angle_step
        self.vel = 0.0
        self.angle = 0.0
        self.angle_goal = 0.0
        self.vel_goal = 0.0
	self.prev_time = 0.0
	self.delta_time = 0.0

    ''' returns fraction of stepsize based on the desired timestep size and the
	difference in milliseconds between the current and last call ''' 
    def deltastep(self, stepsize):
	return self.delta_time/self.timestep * stepsize

    def update_goal_vel_angle(self, vel_goal, angle_goal):
	self.prev_time = datetime().total_seconds * 1000
        self.vel_goal = float(vel_goal)
        self.angle_goal = float(angle_goal)

    def step(self):
	millis = datetime().total_seconds
	self.delta_time = millis * 1000 - self.prev_time
	self.prev_time = millis
	self.vel = self.step_toward_value(self.vel, self.vel_goal, self.deltastep(self.vel_step))
	self.angle = self.step_toward_value(self.angle, self.angle_goal, self.deltastep(self.angle_step))

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
    Control(10)
