import rospy
from navigation_msgs.msg import CartOverride
import time
import random

DELAY_LOWER_BOUND = 2 * 60  # Lower bound in seconds for random delay
DELAY_UPPER_BOUND = 5 * 60  # Upper bound in seconds for random delay
OVERRIDE_MSG_COUNT = 2      # Total amount of override message

# Override message constants
STEERING_JITTER = 0
SUDDEN_BREAK = 1

class RandomMovement(object):

    def __init__(self):
        self.motion_override_pub = rospy.Publisher('/nav_cmd_override', CartOverride, queue_size=10)
        
        while not rospy.is_shutdown():
            time.sleep(random.uniform(DELAY_LOWER_BOUND, DELAY_UPPER_BOUND))
            msg = CartOverride()
            override_type = random.randint(0,OVERRIDE_MSG_COUNT - 1)
            if override_type == STEERING_JITTER:
                msg.steering_jitter = True
            elif override_type == SUDDEN_BREAK:
                msg.sudden_break = True
            self.motion_override_pub.publish(msg)

if __name__ == "__main__":
    try:
        RandomMovement()
    except rospy.ROSInterruptException:
        pass
