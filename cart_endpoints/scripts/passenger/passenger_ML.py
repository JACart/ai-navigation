#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8
from navigation_msgs.msg import VehicleState, Stop, VelAngle
from passengerclassifier import PassengerRFClassifier, PassengerData

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 04/2023
'''
PASSENGER_EDGE_TOP_X_THRESHOLD = 0.5
DRIVER_EDGE_TOP_X_THRESHOLD = -0.7
COUNT_THRESHOLD = 0 # in frames
STOP_COUNT_THRESHOLD = 5
DEPTH_THRESHOLD = 1.3

class Passenger(object):
    def __init__(self):
        rospy.init_node('Passenger_ML')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        
        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/passenger/out_of_bounds', Bool, queue_size=10)
        self.occupants_pub = rospy.Publisher('/passenger/occupants', Int8, queue_size=10)
        self.stop_pub = rospy.Publisher('/passenger/emergency_stop', Bool, queue_size=10)

        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=1)
        self.nav_sub = rospy.Subscriber('/nav_cmd', VelAngle, self.nav_cb, queue_size=10)

        #Private Variables
        self.out_counter = 0
        self.occupants = 0
        self.startingOccupants = 1
        self.stop_counter = 0
        self.stopped = True
        self.prfc = PassengerRFClassifier.load_model("./saved_models/passengerRF_model_255")

        rospy.loginfo("Started pose tracking ML node! (S23)")

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

            if passenger_edge < DRIVER_EDGE_TOP_X_THRESHOLD or driver_edge > PASSENGER_EDGE_TOP_X_THRESHOLD:
                continue
            
            occupant_count += 1

            passenger_keypoints = []
            for kp in person.skeleton_3d.keypoints:
                passenger_keypoints.append(kp.kp)

            passenger_keypoints = np.reshape(np.array([passenger_keypoints]), (-1, 18*3))
            prediction = self.prfc.predict(passenger_keypoints)

            # Classify passengers based on position
            if prediction == 0:
                # Passenger is crossing threshold, signifying an unsafe occupant
                unsafe_person = True

            #else: 
                # Passenger is within the threshold, signifying a safe occupant
             #   print(f"{person.label_id} SAFE")
              #  print(f"driver_edge: {driver_edge}")
            #if driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD and passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD:
             #       occupant_count += 1
            print(f"Person: {person.label_id}")
            #print ("Driver Edge: ", driver_edge, "/", PASSENGER_EDGE_TOP_X_THRESHOLD)
            #print ("Passenger Edge: ", passenger_edge, "/", DRIVER_EDGE_TOP_X_THRESHOLD)
        print(self.occupants, "/", self.startingOccupants)
        self.occupants = occupant_count
                
        # Iterate stop_count
        if not self.stopped and self.startingOccupants > self.occupants:
            self.stop_counter += 1
        else:
            self.stop_counter = 0

        # Iterate out_count
        if not unsafe_person: #and self.has_seen_passanger:
            self.out_counter = 0
        else:
            self.out_counter += 1

        # Publish passenger information
        if self.stop_counter > STOP_COUNT_THRESHOLD and not self.stopped:        # Publish Emergency Stop
            self.stop_pub.publish(True)
            self.stop_counter = 0 
        self.occupants_pub.publish(self.occupants)                          # Publish Occupant Count
        self.out_of_bounds_pub.publish(self.out_counter > COUNT_THRESHOLD)  # Publish Out of Bounds

    ''' 
    Call back method for detecting whether cart is stopped or not 
    ____________
    Args:
        msg:
            message containing a boolean value, T iff. cart is stopped
    '''
    def state_cb(self, msg):
        #print(msg.stopped)
        #self.stopped = msg.stopped
        pass

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