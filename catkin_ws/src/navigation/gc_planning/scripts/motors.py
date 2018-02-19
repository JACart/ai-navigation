import rospy
import serial

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
        speed_ser = serial.Serial(speed_port, 9600)
        rospy.loginfo("Speed serial established")
        """
        #Connect to arduino for steering
        turn_ser = serial.Serial(turn_port, 9600)
        rospy.loginfo("Steering serial established")
        """
        self.speed_string = ''
        current_speed = float(input("Input current speed:"))
        goal_speed = float(input("Input desired speed:"))
        while current_speed<goal_speed+test_increment:
            self.speed_string =  str(goal_speed)+ ',' + str(current_speed)
            print "String being sent: "+self.speed_string
            speed_ser.write(self.speed_string.encode())
            response = speed_ser.readline()
            print "Response: "+response
            current_speed = current_speed +test_increment
            rospy.sleep(0.5)
        rospy.spin()

if __name__ == "__main__":
    
    MotorNode()