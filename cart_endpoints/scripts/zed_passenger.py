#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String

'''
This ROS node keeps track of passenger pose data and determines whether passengers
are inside or outside bounding box.

Authors: Daniel Hassler, Jacob Hataway, Jakob Lindo, Maxwell Stevens
Version: 02/2023
'''
PASSENGER_EDGE_TOP_X_THRESHOLD = 315
DRIVER_EDGE_TOP_X_THRESHOLD = 1000
OUT_COUNT_THRESHOLD = 20 # in frames

class ZedPassenger(object):
    def __init__(self):
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        # Name of coordinate frame of the passenger camera
        self.coordinate_frame = rospy.get_param("coordinate_frame", "/passenger_cam_left_camera_frame")

        # private variables
        self.out_count = 0 # a threshold to determine if a passenger is really outside the vehicle.
        self.persons = [] # a list to keep track of all the persons detected from the objectdetection

        # Publishers
        self.out_of_bounds_pub = rospy.Publisher('/zed_passenger/out_of_bounds', String, queue_size=10)
        # subscribers:
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.received_persons, queue_size=10)

        rospy.init_node('ZedPassenger')
        rospy.loginfo("Started pose tracking node! (S23)")
        rospy.loginfo("Coordinate Frame: %s" % (self.coordinate_frame))

        # Transform service that listens to all links between coodrinate frames.
        self.t = tf.TransformListener()

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            #rospy.Subscriber('/passenger_cam/passenger/obj_det/objects', ObjectsStamped, create_transform_link) # attempting to create transform link
            #self.visualize_2dbox()
            self.publish_out()
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

        for person in people_box:
            self.persons.append(person)
    
    def publish_out(self):
        while len(self.persons) != 0:
            person = self.persons.pop(0)
            person_corners = person.bounding_box_2d.corners
            driver_edge = person_corners[1].kp[0]
            passenger_edge = person_corners[0].kp[0]

            if passenger_edge < PASSENGER_EDGE_TOP_X_THRESHOLD or driver_edge > DRIVER_EDGE_TOP_X_THRESHOLD:
                #print("out")
                self.out_count+=1
                if self.out_count >= OUT_COUNT_THRESHOLD:
                    self.out_of_bounds_pub.publish("Passenger has been out for more than %d time lapses, send message to UI or queue shutdown." % (self.out_count))
            else:
                self.out_count = 0
            self.out_of_bounds_pub.publish("out count: " + str(self.out_count))

    # def visualize_2dbox(self):
    #     self.t.waitForTransform("/map", self.coordinate_frame, rospy.Time(0), rospy.Duration(0.01))

def create_transform_link(data):
    '''
    Attempting to create a transform link map -> passenger_cam_left_camera_frame to visualize bounding box in Rviz
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

