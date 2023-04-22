#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8, String
from navigation_msgs.msg import VehicleState, Stop, VelAngle
from passengerclassifier import PassengerRFClassifier, PassengerData

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 04/2023
'''

# Camera threshold values. Used to filter objects.
PASSENGER_EDGE_THRESHOLD = 0.5
DRIVER_EDGE_THRESHOLD = -0.7
DEPTH_THRESHOLD = 1.3


# Count threshold values. Make sure we are confident before publishing.
UNSAFE_POSE_THRESHOLD = 5
POSE_STOP_THRESHOLD = 40
PASSENGER_EXIT_THRESHOLD = 5


class Passenger(object):
    def __init__(self):
        rospy.init_node('Passenger')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        
        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/passenger/unsafe_pose', Bool, queue_size=10)
        self.occupants_pub = rospy.Publisher('/passenger/occupants', Int8, queue_size=10)
        self.stop_pub = rospy.Publisher('/passenger/emergency_stop', String, queue_size=10)

        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=1)
        self.nav_sub = rospy.Subscriber('/nav_cmd', VelAngle, self.nav_cb, queue_size=1)

        # Confidence Counters
        self.out_counter = 0
        self.stop_counter = 0

        self.occupants = 0
        self.startingOccupants = 0
        self.stopped = True
        self.prfc = PassengerRFClassifier.load_model("./saved_models/passengerRF_model_113")

        rospy.loginfo("Started pose tracking node! (S23)")

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.stopped:
                self.startingOccupants = self.occupants
            r.sleep()
    
    ''' 
    callback method for passenger camera frames
    _______
    Args:
        msg: person objects from the zed camera.
    ________

    '''
    def received_persons(self, msg):
        people = msg.objects
        unsafe_person = False

        # Iterate through and classify detected objects
        occupant_count = 0
        for person in people:
            person_corners = person.bounding_box_3d.corners
            driver_edge = person.skeleton_3d.keypoints[5].kp[1]     # driver's top left side, y point
            passenger_edge = person.skeleton_3d.keypoints[2].kp[1]  # passenger's top right side, y point
            person_depth = person_corners[1].kp[0]                  # occupant's furthest back point, z position

            # Ignore passengers beyond a certain depth
            if person_depth > DEPTH_THRESHOLD:
                continue

            # Ignore passengers beyond a certain y threshold
            if passenger_edge < DRIVER_EDGE_THRESHOLD or driver_edge > PASSENGER_EDGE_THRESHOLD:
                continue
            
            occupant_count += 1

            passenger_keypoints = []
            for kp in person.skeleton_3d.keypoints:
                passenger_keypoints.append(kp.kp)

            passenger_keypoints = np.reshape(np.array([passenger_keypoints]), (-1, 18*3))
            prediction = self.prfc.predict(passenger_keypoints)

            # Classify passengers based on position
            unsafe_person = prediction == 0
            if prediction == 0:
                # Passenger is crossing threshold, signifying an unsafe occupant
                unsafe_person = True

        self.occupants = occupant_count
                
        # Iterate stop_count
        if not self.stopped and self.startingOccupants > self.occupants:
            self.stop_counter += 1
        else:
            self.stop_counter = 0

        # Iterate out_count
        if unsafe_person:
            self.out_counter += 1
        else:
            self.out_counter = 0

        # Publish passenger information
        if self.out_counter > POSE_STOP_THRESHOLD and not self.stopped:
            self.stop_pub.publish("unsafe-pose-stop")                       # Publish unsafe pose Stop
        if self.stop_counter > PASSENGER_EXIT_THRESHOLD and not self.stopped:   # Publish  passenger exit Stop
            self.stop_pub.publish("passenger-exit-stop")
            self.stop_counter = 0 
        self.occupants_pub.publish(self.occupants)                          # Publish Occupant Count
        self.out_of_bounds_pub.publish(self.out_counter > UNSAFE_POSE_THRESHOLD)  # Publish Out of Bounds

    ''' 
    Call back method for detecting whether cart is stopped or not 
    ____________
    Args:
        msg:
            message containing a nav command. 
    '''
    def nav_cb(self, msg):
        self.vel = msg.vel_curr
        if self.vel < .2:
            self.stopped = True
        else:
            self.stopped = False
        

if __name__ == "__main__":
    try:
        Passenger()
    except rospy.ROSInterruptException:
        pass