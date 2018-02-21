#!/usr/bin/env python
import rospy
from navigation_msgs.msg import fe_request, lat_long_point, list_of_goals

wilson_address = "951 Madison Dr. Harrisonburg, VA"
lakeview_address = "298 Port Republic Road, Harrisonburg, VA"
isat_address = "701 Carrier Dr., Harrisonburg, VA"

class TestNode(object):
    
    def __init__(self):
        """ Set up the node. """
        
        rospy.init_node('test_node')
        
        self.pub = rospy.Publisher('/fe_request', fe_request, queue_size = 10)
        request = fe_request()
        request.starting_location = wilson_address
        request.final_location = lakeview_address
        rospy.sleep(5.0)
        rospy.loginfo("sending!"+str(request))
        self.pub.publish(request)

if __name__ == "__main__":
    TestNode()