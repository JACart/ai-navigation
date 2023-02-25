#!/usr/bin/env python
import rospy
import time
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 02/2023
'''
PASSENGER_EDGE_TOP_X_THRESHOLD = 0.6
DRIVER_EDGE_TOP_X_THRESHOLD = -0.8
OUT_COUNT_THRESHOLD = 5 # in frames
DEPTH_THRESHOLD = 0.75

class Passenger(object):
    def __init__(self):
        rospy.init_node('Passenger')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        
        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/passenger/out_of_bounds', Bool, queue_size=10)
        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=10)

        #Private Variables
        self.out_counter = 0

        rospy.loginfo("Started pose tracking node! (S23)")

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
    
    def received_persons(self, msg):
        people = msg.objects
        unsafe_person = False

        # Iterate through detected objects
        for person in people:
            person_corners = person.bounding_box_3d.corners
            driver_edge = person_corners[3].kp[1] # driver's top left side, y point
            passenger_edge = person_corners[0].kp[1] # passenger's top right side, y point
            person_depth = person_corners[1].kp[2] # occupant's furthest back point, z position

            # Ignore passengers beyond a certain depth
            if person_depth > DEPTH_THRESHOLD:
                continue

            # Detect if passengers are crossing threshold. This signifies unsafe.
            if (driver_edge < DRIVER_EDGE_TOP_X_THRESHOLD and passenger_edge > DRIVER_EDGE_TOP_X_THRESHOLD) or (passenger_edge > PASSENGER_EDGE_TOP_X_THRESHOLD and driver_edge < PASSENGER_EDGE_TOP_X_THRESHOLD):
                unsafe_person = True    

        # Iterate out_count and publish true if passenger has been unsafe for too long. Otherwise publish false.
        if not unsafe_person:
            self.out_counter = 0
        else:
            self.out_counter += 1

        if self.out_counter > OUT_COUNT_THRESHOLD:
            self.out_of_bounds_pub.publish(True)
        else:
            self.out_of_bounds_pub.publish(False)
        print(self.out_counter)

if __name__ == "__main__":
    try:
        Passenger()
    except rospy.ROSInterruptException:
        pass