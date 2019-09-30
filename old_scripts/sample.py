#!/usr/bin/env python 

import rospy 
from navigation_msgs.msg import SampleData 
    
class Sample(object):

    def __init__(self):
        rospy.init_node('sample')
        self.sample_pub = rospy.Publisher('/sample', SampleData, queue_size=10, latch=True)
        #self.sample_pub.publish()
        print("testing")
        rospy.spin()

if __name__ == "__main__":
    Sample()
