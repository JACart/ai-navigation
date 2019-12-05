#!/usr/bin/env python

'''
Full safety system prototype for autonomous vehicle.
Checks for passenger safety at the beginning of a trip.
Monitors safety during the trip.
Checks for passenger exit.

version 11.6.19

'''

import cv2
import os
import time
import datetime
import sys
import rospy
import vlc
import numpy as np
import cPickle as pickle
from sklearn.ensemble import RandomForestClassifier
import paho.mqtt.client as mqtt
from std_msgs.msg import Int8, String, Bool
from sensor_msgs.msg import Image
import json
# Add openpose to system PATH and import
sys.path.append(os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/build/python', ''))
from openpose import pyopenpose as op
class pose_tracking(object):
    
    def __init__(self):
        rospy.init_node('pose_tracker')
        rospy.loginfo("Started pose tracking node!")
        self.passenger_safe_pub = rospy.Publisher('/passenger_safe', Bool, queue_size=10)
        self.passenger_exit_pub = rospy.Publisher('/passenger_exit', Bool, queue_size=10)
        self.safety_intial_sub = rospy.Subscriber('/location_speech', Bool, self.initial_safety)
        #self.safety_constant_sub = rospy.Subscriber('/safety_constant', Bool, self.safety_analysis)
        self.safety_exit_sub = rospy.Subscriber('/safety_exit', Bool, self.passenger_exit)
        self.image_raw_sub = rospy.Subscriber('/camera/image_raw', Image, self.update_image)
        
        ###################################################
        # Set up global variables for use in all methods. #
        ###################################################
        self.start_time = time.time()
        self.start_time_stamp = datetime.datetime.now()
        
        # Setup logging file
        self.path = os.path.expanduser("~") + "/catkin_ws/src/ai-navigation/cart_endpoints/scripts/logs/"
        if os.path.isdir(self.path):
            pass
        else:
            os.makedirs(self.path)
        self.full_path = self.path + str(self.start_time_stamp.date()) + "_" + str(self.start_time_stamp.time())
        os.makedirs(self.full_path)
        self.f = open(self.full_path + "/log.txt", "wa")
        self.f.write("System booted: {}\n".format(self.start_time_stamp))

        self.CONFIDENCE_THRESHOLD = 15
        self.trip_live = False
        self.image_raw = None
        self.image_ready = False
        self.enter_sound = vlc.MediaPlayer(os.path.join(os.path.expanduser("~"), "catkin_ws/src/ai-navigation/cart_endpoints/sounds/", "enter.mp3"))

        
        ######################
        ### OpenPose Setup ###
        ######################
        # Custom Params (refer to include/openpose/flags.hpp for more parameters)
        self.params = dict()
        self.params["model_folder"] = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/openpose/models/', '')

        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()
        self.initial_safety()
        rate = rospy.Rate(5)
        self.enter_sound.play()
        while not rospy.is_shutdown():
            rate.sleep()
        
    #######################
    ### ROS Topic Stuff ###
    #######################

    def sendPassengerUnsafe(self):
        self.passenger_safe_pub.publish(False)


    def sendPassengerSafe(self):
        self.passenger_safe_pub.publish(True)


    def sendPassengerExit(self):
        self.passenger_exit_pub.publish(True)
        
    def update_image(self, msg):
        self.image_raw = np.frombuffer(msg.data, dtype=np.uint8)
        self.image_raw.shape = (msg.height, msg.width, 3)
        self.image_ready = True


    # #######################################


    ######################
    # Safety Monitoring Methods
    #################

    # Live analysis during trip
    def safety_analysis(self):
        confidence = 0
        self.trip_live = True
        while self.trip_live and not rospy.is_shutdown():
            pred, frame_analyzed, op_output = self.analyze()

            # Check passenger safety and update confidence level
            if (pred[0] == 1 and confidence < (self.CONFIDENCE_THRESHOLD*2)):
                confidence += 1
            else:
                if confidence > 0:
                    confidence -= 1
                else:
                    confidence = 0

            # Check confidence against threshold and call unsafe method if necessary.
            if confidence >= self.CONFIDENCE_THRESHOLD:
                cur_time = time.time()
                cur_time = round(cur_time - self.start_time,2)
                # log OpenPose keypoints for current frame.
                self.f.write("Unsafe situation detected.\nTimestamp: \n{}sec\n".format(cur_time))
                self.f.write("OpenPose frame keypoint data:\n")
                self.f.write(str(frame_analyzed) + "\n\n")
                cv2.imwrite(self.full_path + "/frame%.2f.jpg" % cur_time, op_output)
                # Send unsafe message
                self.sendPassengerUnsafe()
            else:
                self.sendPassengerSafe()
            if cv2.waitKey(1) == 27:
                break  # esc to quit

            # Visualize confidence changes
            sys.stdout.write("\r confidence level: [ {} ] \t".format(confidence))
        cv2.destroyAllWindows()
        

    # Single step of analysis. Analyze one frame and return safety classification.
    def analyze(self):
        rate = rospy.Rate(5)
        while self.image_ready == False and not rospy.is_shutdown():
            rate.sleep()
        final_image = self.image_raw
        
        final_image = self.edit_video(final_image)
        #print(final_image)
        ###############
        # Process openpose
        ############
        # Process image
        datum = op.Datum()
        datum.cvInputData = final_image
        self.opWrapper.emplaceAndPop([datum])

        # Display Image
        frame = datum.poseKeypoints
        cv2.imshow("Frame", datum.cvOutputData)

        ############
        # Classify current passenger state.
        ######
        prediction = self.safety_check(frame)
        
        
        return prediction, frame, datum.cvOutputData



    #Analyze passenger safety.
    # Returns np array
    def safety_check(self, frame):
        
        filename = os.path.join(os.path.expanduser("~"), 'catkin_ws/src/ai-navigation/cart_endpoints/scripts/', 'py2.7full_dataset_model.sav')
        loaded_model = pickle.load(open(filename, 'rb'))
        if (len(frame.shape) == 0):
            frame = np.zeros((1,75))
            
        else:
            # TODO: Fix for multiple people in frame.
            frame = np.reshape(frame, (-1,75))


        return loaded_model.predict(frame)


    # Method called at the beginning of the trip to make sure the passenger is safely in the vehicle.
    # Once passenger safety is determined, a signal will be sent to start the trip.
    def initial_safety(self):
        safety_counter = 0
            
        while safety_counter < 30 and not rospy.is_shutdown():
            pred, frame, op_output = self.analyze()
            if pred.size > 0:
                if pred[0] == 0:
                    safety_counter += 1
                else:
                    if safety_counter > 0:
                        safety_counter -= 1
                    else:
                        safety_counter = 0
            else:
                if pred == 0:
                    safety_counter += 1
                else:
                    if safety_counter > 0:
                        safety_counter -= 1
                    else:
                        safety_counter = 0

            if cv2.waitKey(1) == 27:
                break  # esc to quit
            
            # Visualize safety counter
            sys.stdout.write("\r counter: [ {} ] \t".format(safety_counter))
        
        self.f.write("Initial safety established: {}".format(datetime.datetime.now()))
        self.sendPassengerSafe()
        print("\nPASSENGER SAFE")
        self.enter_sound.stop()
        self.enter_sound.play()
        self.safety_analysis()
        

    # Method called at the end of the trip to ensure the passenger exits the vehicle safely.
    def passenger_exit(self):
        self.trip_live = False
        exit = 0
        while exit <= 30*4  and not rospy.is_shutdown():
            rate = rospy.Rate(5)
            while self.image_ready == False and not rospy.is_shutdown():
                rate.sleep()
            final_image = self.image_raw

            final_image = self.edit_video(final_image)

            ###############
            # Process openpose
            ############
            # Process image
            datum = op.Datum()
            datum.cvInputData = final_image
            self.opWrapper.emplaceAndPop([datum])

            cv2.imshow("Frame", datum.cvOutputData)

            # If no person is detected in the frame then update exit counter
            # If a person is detected, check their position to see if they are clear of the cart.
            if (datum.poseKeypoints.ndim > 0): 
                people = datum.poseKeypoints.size/75
                index = 0
                while index < people:
                    # Check sides of cart to make sure people are clear. 
                    if datum.poseKeypoints[index,1,0] < 50 or datum.poseKeypoints[index,1,0] > 600:
                        exit+=1
                    else:
                        if exit <= 0: 
                            exit = 0 
                        else:
                            exit -= 1
                    index+=1
            else:
                exit+=1

            if cv2.waitKey(1) == 27:
                break  # esc to quit

            # Visualize exit counter
            sys.stdout.write("\r counter: [ {} ] \t".format(exit))
        cv2.destroyAllWindows()

        
        print("PASSENGER SAFELY EXITED")  
        cur_time = time.time()
        
        self.f.write("Trip ended successfully.\n")
        self.f.write("Trip time: {} sec\n".format(round((cur_time - self.start_time), 2)))    
        self.sendPassengerExit()


    # Edit video frame for proper processing.
    def edit_video(self, img):
        # Edit video
        h,w = img.shape[:2]
        cropped = img[:, :int(w/2)]
        final_image = cv2.flip(cropped, -1)
        return final_image

    # Method called when passenger is unsafe.
    # Sends signal to monitoring/UI
    def unsafe(self):
        self.sendPassengerUnsafe()
        print("PASSENGER UNSAFE")
        
    def safe(self):
        self.sendPassengerSafe()
        
        

if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass

