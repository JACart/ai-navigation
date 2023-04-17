#!/usr/bin/env python
import rospy
import time
import sklearn
from zed_interfaces.msg import ObjectsStamped
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from queue import PriorityQueue
import numpy as np

'''
This ROS node collects passenger data for ML use.

Authors: Daniel Hassler, et. al.
Version: 02/2023
'''
DEPTH_THRESHOLD = 1.3

class PassengerML(object):
    def __init__(self):
        rospy.init_node('PassengerML_Data_Collect')
        
        # Topic that object detection data from passenger camera is coming from
        self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        
        # Publishers
        # self.out_of_bounds_pub = rospy.Publisher('/passenger/out_of_bounds', Bool, queue_size=10)

        # Subscribers:
        
        # IMPORTANT: make sure to comment these out if you don't want to collect data
        response = input("Do you want to add onto the in_bounds training data (I), out_bounds data (O), or none at all (N)? (I/O/N): ")
        self.in_bounds_sub = None
        self.out_bounds_sub = None

        if response == "I":
            self.in_bounds_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.data_collect_in_bounds, queue_size=10)
        elif response == "O":
            self.out_bounds_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.data_collect_out_bounds, queue_size=10)
        else:
            pass

        #Private Variables
        # self.in_bounds_X_data = np.load("./passenger_ML_data/in_bounds_X_data.npy").tolist()
        # self.in_bounds_y_data = np.load("./passenger_ML_data/in_bounds_y_data.npy").tolist()
        self.in_bounds_X_data = []
        self.in_bounds_y_data = []
        # self.out_bounds_X_data = np.load("./passenger_ML_data/out_bounds_X_data.npy").tolist()
        # self.out_bounds_y_data = np.load("./passenger_ML_data/out_bounds_y_data.npy").tolist()
        self.out_bounds_X_data = []
        self.out_bounds_y_data = []
        rospy.loginfo("Started data collection/ML node! (S23)")

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()
    
    def data_collect_in_bounds(self, msg):
        '''
        Used to collect passenger data for every callback in the object detection topic.
        Greddily assigning 1 (SAFE) to data it detects in the cart as I am INSIDE the golf cart moving slightly
        in a safe manner..

        '''
        curr_entry = [] # shape (18,3)
        if msg.objects[0].label == "Person":
            person = msg.objects[0]
            person_corners = person.bounding_box_3d.corners
            person_depth = person_corners[1].kp[0]

            if person_depth > DEPTH_THRESHOLD:
                return
                
            for i,kp in enumerate(msg.objects[0].skeleton_3d.keypoints):
                curr_entry.append(kp.kp) # appends keypoint x,y,z values to list.

            self.in_bounds_y_data.append(1)
            self.in_bounds_X_data.append(curr_entry)

        print("X: ", np.array(self.in_bounds_X_data).shape) # shape (num_entries, 18, 3)
        print("y: ", np.array(self.in_bounds_y_data).shape) # shape (num_entries,)

    def data_collect_out_bounds(self, msg):
        '''
        Used to collect passenger data for every callback in the object detection topic.
        Greddily assigning 0 (UNSAFE) to data it detects in the cart as I am OUTSIDE/partially outside 
        the golf cart moving slightly in an unsafe manner.

        '''
        curr_entry = [] # shape (18,3)
        if msg.objects[0].label == "Person":
            for i,kp in enumerate(msg.objects[0].skeleton_3d.keypoints):
                curr_entry.append(kp.kp) # appends keypoint x,y,z values to list.
            self.out_bounds_y_data.append(0)
            self.out_bounds_X_data.append(curr_entry)
        print("X: ", np.array(self.out_bounds_X_data).shape) # shape (num_entries, 18, 3)
        print("y: ", np.array(self.out_bounds_y_data).shape) # shape (num_entries,)

class PassengerMLRandomForest():
    def __init__(self):
        self.in_bounds_X_data = np.load("./passenger_ML_data/in_bounds_X_datav04172023.npy")
        self.in_bounds_y_data = np.load("./passenger_ML_data/in_bounds_y_datav04172023.npy")
        self.out_bounds_X_data = np.load("./passenger_ML_data/out_bounds_X_datav04172023.npy")
        self.out_bounds_y_data = np.load("./passenger_ML_data/out_bounds_y_datav04172023.npy")
        print(self.in_bounds_X_data.shape)
        print(self.in_bounds_y_data.shape)
        print(self.out_bounds_X_data.shape)
        print(self.out_bounds_y_data.shape)

if __name__ == "__main__":
    try:
        pml = PassengerML()

        if np.array(pml.in_bounds_X_data).shape[0] > 0:
            with open('./passenger_ML_data/in_bounds_X_datav04172023.npy', 'wb') as f:
                np.save(f, np.array(pml.in_bounds_X_data))
            with open('./passenger_ML_data/in_bounds_y_datav04172023.npy', 'wb') as f:
                np.save(f, np.array(pml.in_bounds_y_data))

        if np.array(pml.out_bounds_X_data).shape[0] > 0:
            with open('./passenger_ML_data/out_bounds_X_datav04172023.npy', 'wb') as f:
                np.save(f, np.array(pml.out_bounds_X_data))
            with open('./passenger_ML_data/out_bounds_y_datav04172023.npy', 'wb') as f:
                np.save(f, np.array(pml.out_bounds_y_data))
        
        pmlrf = PassengerMLRandomForest()

    except rospy.ROSInterruptException:
        pass