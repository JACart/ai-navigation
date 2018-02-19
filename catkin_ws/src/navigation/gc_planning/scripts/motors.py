import rospy
import serial
from navigation_msgs.msg import vel_angle

speed_port = '/dev/ttyACM0'
turn_port = ''
test_increment = 0.1
global current_speed
global goal_speed

class MotorNode(object):
    
    def __init__(self):
        """ Set up the node. """
        rospy.init_node('motor_node')
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
        
        #Code for testing
        response = "No response yet"
        self.current_speed = float(input("Input current speed:"))
        self.goal_speed = float(input("Input desired speed:"))
        while self.current_speed<self.goal_speed+test_increment:
            self.speed_string =  str(self.goal_speed)+ ',' + str(self.current_speed)
            print "String being sent: "+self.speed_string
            self.speed_ser.write(self.speed_string.encode())
            response = self.speed_ser.readline()
            print "Response: "+response
            self.current_speed = self.current_speed +test_increment
            rospy.sleep(0.5)
        rospy.loginfo("Testing complete")
        #Code for using subscriber
        self.motion_subscriber = rospy.Subscriber('/vel_angle', vel_angle, self.motion_callback, queue_size = 10)
        rospy.spin()
        
    def motion_callback(planned_vel_angle):
        velocity = planned_vel_angle.vel
        angle  = planned_vel_angle.angle
        self.speed_string =  str(self.goal_speed)+ ',' + str(self.current_speed)
        self.speed_ser.write(self.speed_string.encode())
        rospy.loginfo("String being sent: "+self.speed_string)
        
if __name__ == "__main__": 
    MotorNode()
