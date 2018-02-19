import rospy
import serial
from navigation_msgs.msg import vel_angle

speed_port = '/dev/ttyACM0'
turn_port = ''

class MotorEndpoint(object):
    
    def __init__(self):
        global speed_ports
        self.killswitch = false
        self.current_speed = 0.0
        self.goal_speed = 0.0
        self.goal_angle = 0.0
        """ Set up the node. """
        rospy.init_node('motor_endpoint')
        rospy.loginfo("Starting motor node!")
        #Connect to arduino for sending speed
        self.speed_ser = serial.Serial(speed_port, 9600)
        rospy.loginfo("Speed serial established")
        """
        #Connect to arduino for steering
        turn_ser = serial.Serial(turn_port, 9600)
        rospy.loginfo("Steering serial established")
        """
        self.speed_string = ''
        response = "No response yet"
        self.motion_subscriber = rospy.Subscriber('/nav_cmd', vel_angle, self.motion_callback, queue_size = 10)
        self.killswitch_subscriber = rospy.Subscriber('/emergency_stop', emergency_stop, self.kill, queue_size = 10)
        rospy.spin()
        
    def motion_callback(self, planned_vel_angle):
        if self.killswitch:
            self.speed_ser.write("0.0, 0.0".encode())
        else
            self.goal_speed = planned_vel_angle.vel
            self.goal_angle = planned_vel_angle.angle
            self.current_speed = planned_vel_angle.vel_curr
            self.speed_string =  str(self.goal_speed)+ ',' + str(self.current_speed)
            self.speed_ser.write(self.speed_string.encode())
            rospy.loginfo("String being sent: "+self.speed_string)
        
    def kill(self, data):
        self.killswitch = data.emergency_stop
        
        
if __name__ == "__main__": 
    try:
        MotorEndpoint()
    except rospy.ROSInterruptException:
        pass
