import rospy
from navigation_msgs.msg import CartOverride
import time
import random

DELAY_LOWER_BOUND = 25 #* 60  # Lower bound in seconds for random delay
DELAY_UPPER_BOUND = 26 #* 60  # Upper bound in seconds for random delay
OVERRIDE_MSG_COUNT = 2      # Total amount of override message

# Override message constants
STEERING_JITTER = 0
SUDDEN_BRAKE = 1

class RandomMovement(object):

    def __init__(self):

        rospy.init_node("nav_cmd_override")
        self.motion_override_pub = rospy.Publisher('/nav_cmd_override', CartOverride, queue_size=10)
        
        while not rospy.is_shutdown():
            time.sleep(random.uniform(DELAY_LOWER_BOUND, DELAY_UPPER_BOUND))
            msg = CartOverride()
            override_type = 1 #random.randint(0,OVERRIDE_MSG_COUNT - 1)
            if override_type == STEERING_JITTER:
                msg.steering_jitter = True
            elif override_type == SUDDEN_BRAKE:
                msg.sudden_brake = True
            self.motion_override_pub.publish(msg)

if __name__ == "__main__":
    try:
        RandomMovement()
    except rospy.ROSInterruptException:
        pass
