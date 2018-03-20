#/usr/bin/env python

from datetime import datetime, timedelta
import math

angle_step = 1.0 #per timestep

''' uses a linear acceleration function '''
class Controller(object):
    def __init__(self, timestephz):
	global angle_step
	self.timestep = (1.0/float(timestephz))*1000 #convert hz to millis per timestep
        self.vel_step = 0.0
        self.angle_step = angle_step
        self.vel = 0.0
        self.angle = 0.0
        self.angle_goal = 0.0
        self.vel_goal = 0.0
	self.prev_time = self.millis_since_epoch()
	self.delta_time = 0.0

    def millis_since_epoch(self):
	return (datetime.now()-datetime(1970,1,1)).total_seconds() * 1000
	
    ''' sets the acceleration toward a goal velocity 
	(velocity step size in terms of timesteps rather than seconds) '''
    def accelerate(self, acceleration, goal_velocity):
	seconds = self.timestep/1000.0	 #seconds per timestep
	self.vel_step = acceleration*seconds #meters per timestep
	self.vel_goal = goal_velocity

    ''' returns fraction of stepsize based on the desired timestep size and the
	difference in milliseconds between the current and last call ''' 
    def deltastep(self, stepsize):
	return (self.delta_time/self.timestep) * stepsize

    ''' to be called at each timestep '''     
    def step(self):
	millis = self.millis_since_epoch()
	self.delta_time = millis - self.prev_time
	self.prev_time = millis
	self.vel = self.step_toward_value(self.vel, self.vel_goal, self.deltastep(self.vel_step))
	self.angle = self.step_toward_value(self.angle, self.angle_goal, self.deltastep(self.angle_step))

    def get_velocity(self):
	return self.vel

    def get_angle(self):
	return self.angle

    ''' steps toward a dest value without going past it. If the current value
	is higher, the step is subtracted, if it is lower, the step is added. '''
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
