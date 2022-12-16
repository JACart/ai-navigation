import rospy
from navigation_msgs.msg import CartOverride
import time
import random
import sys

DELAY_LOWER_BOUND = 2 * 60  # Default lower bound in seconds for random delay
DELAY_UPPER_BOUND = 3 * 60  # Default upper bound in seconds for random delay
OVERRIDE_MSG_COUNT = 2      # Total amount of override message

# Override message constants
STEERING_JITTER = 0
SUDDEN_BRAKE = 1

class RandomEvents(object):

    def __init__(self):
        if len(sys.argv) == 3 and sys.argv[1] != '' and sys.argv[2] != '': 
            global DELAY_LOWER_BOUND
            DELAY_LOWER_BOUND = int(sys.argv[1])
            global DELAY_UPPER_BOUND 
            DELAY_UPPER_BOUND = int(sys.argv[2])

        rospy.init_node("nav_cmd_override")
        self.motion_override_pub = rospy.Publisher('/nav_cmd_override', CartOverride, queue_size=10)
        
        while not rospy.is_shutdown():
            time.sleep(random.uniform(DELAY_LOWER_BOUND, DELAY_UPPER_BOUND))
            msg = CartOverride()
            override_type = random.randint(0,OVERRIDE_MSG_COUNT - 1)
            if override_type == STEERING_JITTER:
                msg.steering_jitter = True
                t = time.localtime()
                current_time = time.strftime("%H:%M:%S", t)
                print("(", current_time, ") Sending Steering Jitter")

            elif override_type == SUDDEN_BRAKE:
                msg.sudden_brake = True
                t = time.localtime()
                current_time = time.strftime("%H:%M:%S", t)
                print("(", current_time, ") Sending Sudden Brake")
            self.motion_override_pub.publish(msg)

if __name__ == "__main__":
    try:
        RandomEvents()
    except rospy.ROSInterruptException:
        pass
