#!/usr/bin/env python
import rospy
import tf
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 02/2023
'''
PASSENGER_EDGE_TOP_X_THRESHOLD = 315
DRIVER_EDGE_TOP_X_THRESHOLD = 1000
OUT_COUNT_THRESHOLD = 10

class ZedPassenger(object):
    def __init__(self):
        rospy.init_node('ZedPassenger')
        rospy.loginfo("Started pose tracking node! (S23)")

        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        print(self.objects_in)

        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=10)
        

        self.out_count = 0 # a threshold to determine if a passenger is really outside the vehicle.
        
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

    def received_persons(self, msg):
        '''
              This callback takes in data subscribed from the object detection topic and preforms
              calculations on the data.

              Here is our 2D bounding box representation:
              
              0 ------- 1
              |         |
              |         |
              |         |
              3 ------- 2
              zed_interfaces/Keypoint2Di[4] corners

              Helpful resource: https://www.stereolabs.com/docs/ros/object-detection/
        '''

        people_box = msg.objects # a list of all persons detected

        for i, person in enumerate(people_box):
            person_corners = person.bounding_box_2d.corners
            driver_edge = person_corners[1].kp[0]
            passenger_edge = person_corners[0].kp[0] 

            if passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD or driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD:
                if self.out_count >= OUT_COUNT_THRESHOLD:
                    print("Passenger %d has been out for more than %d time lapses, send message to UI or queue shutdown" % (i, self.out_count))
                print("out")
                self.out_count+=1
            else:
                self.out_count = 0
                print("in")


if __name__ == "__main__":
    try:
        ZedPassenger()
    except rospy.ROSInterruptException:
        pass

