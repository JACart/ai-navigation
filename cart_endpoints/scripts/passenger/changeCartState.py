from navigation_msgs.msg import VehicleState
import rospy

class Main(object):
    def __init__(self):
        rospy.init_node('temp_cart_state_changer')

        self.pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size=10)
        msg = VehicleState()
        
        while True:
            s = input("input d or s:\n")
            if s == 'd':
                msg.stopped = False
                self.pub.publish(msg)
                print("Cart state changed to Driving\n")
            elif s == 's':
                msg.stopped = True
                self.pub.publish(msg)
                print("Cart state changed to Stopped\n")
            else:
                exit()

if __name__ == "__main__":
    try:
        Main()
    except rospy.ROSInterruptException:
        pass