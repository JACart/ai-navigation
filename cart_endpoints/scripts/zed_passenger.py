#!/usr/bin/env python
import rospy
import tf
import tf2_ros
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
OUT_COUNT_THRESHOLD = 20 # in frames
DEPTH_THRESHOLD = 0.75

class ZedPassenger(object):
    def __init__(self):
        rospy.init_node('ZedPassenger')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        # Name of coordinate frame of the passenger camera
        self.coordinate_frame = rospy.get_param("coordinate_frame", "/passenger_cam_left_camera_frame")

        # private variables
        self.persons = {} # a dictionary of all the persons keeping track of consecutive times out of bounds

        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/zed_passenger/out_of_bounds', Bool, queue_size=10)
        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=10)

        rospy.loginfo("Started pose tracking node! (S23)")
        rospy.loginfo("Coordinate Frame: %s" % (self.coordinate_frame))

        # Transform service that listens to all links between coodrinate frames.
        self.t = tf.TransformListener()

        r = rospy.Rate(5)#loginfo
        while not rospy.is_shutdown():
            #rospy.Subscriber('/passenger_cam/passenger/obj_det/objects', ObjectsStamped, create_transform_link) # attempting to create transform link
            #self.visualize_2dbox()
            #self.publish_out()
            r.sleep()

    def received_persons(self, msg):
        '''
              This callback takes in data subscribed from the object detection topic and preforms
              calculations on the data.

              Here's our 3D box representation:

                      1 ------- 2
                     /.        /|
                    0 ------- 3 |
                    | .       | |
                    | 5.......| 6
                    |.        |/
                    4 ------- 7
              zed_interfaces/Keypoint3D[8] corners

              Helpful resource: https://www.stereolabs.com/docs/ros/object-detection/
        '''

        people_box = msg.objects
        for person in people_box:
            person_corners = person.bounding_box_3d.corners
            # print("Keypoint 0: ")
            # print(person_corners[0])
            # print("Keypoint 1: ")
            # print(person_corners[1])
            # print("Keypoint 2: ")
            # print(person_corners[2])
            # print("Keypoint 3: ")
            # print(person_corners[3])
            driver_edge = person_corners[3].kp[1] # driver's top left side, y point
            passenger_edge = person_corners[0].kp[1] # passenger's top right side, y point
            person_depth = person_corners[1].kp[2] # occupant's furthest back point, z position

            # print("Person Depth: %f " % (person_depth))

            if person.sublabel == "Person" and person.label_id not in self.persons:
                curr_time = time.time()
                # dictionary where key is the person_id and value is (out_count_consecutive_times, current_time)
                self.persons[person.label_id] = (0, curr_time)

            # removes old data from dictionary to only keep track of currently detected people.
            while len(people_box) < len(self.persons):
                # removes the data based on oldest updated time
                sorted_persons = sorted(self.persons.values(), key=lambda x:x[1])
                # print("Sorted persons: ", sorted_persons)
                lost_person = sorted_persons[0]
                self.persons.pop(self.persons.keys()[self.persons.values().index(lost_person)])

            curr_time = time.time()

            if (passenger_edge >= PASSENGER_EDGE_TOP_X_THRESHOLD or driver_edge <= DRIVER_EDGE_TOP_X_THRESHOLD) and person_depth <= DEPTH_THRESHOLD:
                self.persons[person.label_id] = (self.persons[person.label_id][0] + 1, curr_time)
            else:
                self.persons[person.label_id] = (0, curr_time)
            
                           
        print(self.persons)
        # TO DO: publish data to UI (now we have multi-passenger detection)


    
    def publish_out(self):
        """
        While there is input data from zeds unparsed, parse the 
        zed data, check to see an person is within an predefined 
        2d bounding box (from the object detection topic), and
        publishes to (/zed_passenger/out_of_bounds) if the passanger
        has been out of bounds longer then the maximum allowed time frame.
        """
        self.oob = False
        person = self.persons.pop(0)
        person_corners = person.bounding_box_2d.corners
        driver_edge = person_corners[1].kp[0]
        passenger_edge = person_corners[0].kp[0]
        if passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD or driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD:
            self.oob = True
        self.out_of_bounds_pub.publish(self.oob)
        '''
        while len(self.persons) != 0:
            person = self.persons.pop(0)
            person_corners = person.bounding_box_2d.corners
            driver_edge = person_corners[1].kp[0]
            passenger_edge = person_corners[0].kp[0]
            if passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD or driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD:
                oob = True
               # self.out_count+=1
                #if self.out_count >= OUT_COUNT_THRESHOLD:
                #    self.out_of_bounds_pub.publish("Passenger has been out for more than %d time lapses, send message to UI or queue shutdown." % (self.out_count))
           # else:
            #    self.out_count = 0
            #self.out_of_bounds_pub.publish("out count: " + str(self.out_count))
        self.out_of_bounds_pub.publish(oob)'''

    # def visualize_2dbox(self):
    #     self.t.waitForTransform("/map", self.coordinate_frame, rospy.Time(0), rospy.Duration(0.01))

def create_transform_link(data):
    '''
    Attempting to create a transform link map -> passenger_cam_left_camera_frame to visualize bounding box in Rviz
    Currently Depreciated
    '''
    tf2broadcast = tf2_ros.TransformBroadcaster()
    tf2stamp = TransformStamped()
    tf2stamp.header.stamp = rospy.Time.now()
    tf2stamp.header.frame_id = 'map'
    tf2stamp.child_frame_id = 'passenger_cam_left_camera_frame'
    tf2stamp.transform.translation = data.objects[0].position
    print(tf2stamp)
    tf2broadcast.sendTransform(tf2stamp)

if __name__ == "__main__":
    try:
        ZedPassenger()
    except rospy.ROSInterruptException:
        pass

