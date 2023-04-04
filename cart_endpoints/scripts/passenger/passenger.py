#!/usr/bin/env python
import rospy
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8
from navigation_msgs.msg import VehicleState

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 04/2023
'''
PASSENGER_EDGE_TOP_X_THRESHOLD = 0.5
DRIVER_EDGE_TOP_X_THRESHOLD = -0.7
COUNT_THRESHOLD = 5 # in frames
DEPTH_THRESHOLD = 1.5

class Passenger(object):
    def __init__(self):
        rospy.init_node('Passenger')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        
        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/passenger/out_of_bounds', Bool, queue_size=10)
        self.occupants_pub = rospy.Publisher('/passenger/occupants', Int8, queue_size=10)
        self.stop_pub = rospy.Publisher('/passenger/emergency_stop', Bool, queue_size=10)

        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=10)
        self.state_sub = rospy.Subscriber('/vehicle_state', VehicleState, callback=self.state_cb, queue_size=10)

        #Private Variables
        self.out_counter = 0
        self.occupants = 0
        self.startingOccupants = 1
        self.stop_counter = 0
        self.stopped = False

        rospy.loginfo("Started pose tracking node! (S23)")

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.stopped:
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

            # Classify passengers based on position
            if (driver_edge < DRIVER_EDGE_TOP_X_THRESHOLD  \
                    and passenger_edge > DRIVER_EDGE_TOP_X_THRESHOLD) \
                    or (passenger_edge > PASSENGER_EDGE_TOP_X_THRESHOLD \
                    and driver_edge < PASSENGER_EDGE_TOP_X_THRESHOLD):
                # Passenger is crossing threshold, signifying an unsafe occupant
                unsafe_person = True
                occupant_count += 1
                self.has_seen_passanger = True

            elif (driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD and passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD): 
                # Passenger is within the threshold, signifying a safe occupant
                occupant_count += 1
        self.occupants = occupant_count
                
        # Iterate stop_count
        if not self.stopped and self.startingOccupants > self.occupants:
            self.stop_counter += 1
        else:
            self.stop_counter = 0

        # Iterate out_count
        if not unsafe_person and self.has_seen_passanger:
            self.out_counter = 0
        else:
            self.out_counter += 1

        # Publish passenger information
        self.stop_pub.publish(self.stop_counter > COUNT_THRESHOLD)          # Publish Emergency Stop
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
        self.stopped = msg.stopped
        

if __name__ == "__main__":
    try:
        Passenger()
    except rospy.ROSInterruptException:
        pass