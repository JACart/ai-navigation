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
import numpy as np
import pickle
from sklearn.ensemble import RandomForestClassifier
import paho.mqtt.client as mqtt
import json

# Add openpose to system PATH and import
sys.path.append('/home/jacart/catkin_ws/src/openpose/build/python');
from openpose import pyopenpose as op

#######################
# Set up global variables for use in all methods.
##############
cam = cv2.VideoCapture(0)
start_time = time.time()
start_time_stamp = datetime.datetime.now()
# Setup logging file
path = os.getcwd() + "/logs/"
if os.path.isdir(path):
    pass
else:
    os.makedirs(path)
full_path = path + str(start_time_stamp.date()) + "_" + str(start_time_stamp.time())
os.makedirs(full_path)

f = open(full_path + "/log.txt", "x")
f.write("System booted: {}\n".format(start_time_stamp))

CONFIDENCE_THRESHOLD = 15
trip_live = False
#################
# OpenPose Setup
##############
# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "/home/jacart/catkin_ws/src/openpose/models/"

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

######################
# Network Client
###########

def onConnect():
    print('mqtt connected')


def onMessage(data):
    parsedData = json.loads(data)
    if(parsedData['command'] == 'passenger_ready'):
        print('passenger is ready')
        trip_live = True
        initial_safety()
        safety_analysis()
        
    elif(parsedData['command'] == 'passenger_stop'):
        print('passenger stopped the cart')
        trip_live = False
        passenger_exit()

client = mqtt.Client()
client.on_connect = onConnect
client.on_message = onMessage
#client.connect("localhost", 1883, 60)


def sendPassengerUnsafe():
    data = {
        "command": "passenger_unsafe",
        "data": "passenger_unsafe"
    }
    client.publish("/pose", json.dumps(data))


def sendPassengerSafe():
    data = {
        "command": "passenger_safe",
        "data": "passenger_safe"
    }
    client.publish("/pose", json.dumps(data))


def sendPassengerExit():
    data = {
        "command": "passenger_exit",
        "data": "passenger_exit"
    }
    client.publish("/pose", json.dumps(data))

#######################################


######################
# Safety Monitoring Methods
#################

# Live analysis during trip
def safety_analysis():
    confidence = 0
    while trip_live:
        pred, frame_analyzed, op_output = analyze()

        # Check passenger safety and update confidence level
        if (pred[0] == 1 and confidence < CONFIDENCE_THRESHOLD):
            confidence += 1
        else:
            if confidence > 0:
                confidence -= 1
            else:
                confidence = 0

        # Check confidence against threshold and call unsafe method if necessary.
        if confidence >= CONFIDENCE_THRESHOLD:
            cur_time = time.time()
            cur_time = round(cur_time - start_time,2)
            # log OpenPose keypoints for current frame.
            f.write("Unsafe situation detected.\nTimestamp: \n{}sec\n".format(cur_time))
            f.write("OpenPose frame keypoint data:\n")
            f.write(str(frame_analyzed) + "\n\n")
            cv2.imwrite(full_path + "/frame%.2f.jpg" % cur_time, op_output)
            # Send unsafe message
            unsafe()

        if cv2.waitKey(1) == 27:
            break  # esc to quit

        # Visualize confidence changes
        sys.stdout.write("\r confidence level: [ {} ] \t".format(confidence))
    cv2.destroyAllWindows()
    

# Single step of analysis. Analyze one frame and return safety classification.
def analyze():
    ret_val, final_image = cam.read()
    

    final_image = edit_video(final_image)

    ###############
    # Process openpose
    ############
    # Process image
    datum = op.Datum()
    datum.cvInputData = final_image
    opWrapper.emplaceAndPop([datum])

    # Display Image
    frame = datum.poseKeypoints
    cv2.imshow("Frame", datum.cvOutputData)

    ############
    # Classify current passenger state.
    ######
    prediction = safety_check(frame)
    
    
    return prediction, frame, datum.cvOutputData



#Analyze passenger safety.
# Returns np array
def safety_check(frame):
    filename = 'full_dataset_model.sav'
    loaded_model = pickle.load(open(filename, 'rb'))
    if (len(frame.shape) == 0):
        frame = np.zeros((1,75))
        
    else:
        # TODO: Fix for multiple people in frame.
        frame = np.reshape(frame, (-1,75))


    return loaded_model.predict(frame)


# Method called at the beginning of the trip to make sure the passenger is safely in the vehicle.
# Once passenger safety is determined, a signal will be sent to start the trip.
def initial_safety():
    safety_counter = 0
        
    while safety_counter < 30:
        pred, frame, op_output = analyze()
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
    
    f.write("Initial safety established: {}".format(datetime.datetime.now()))
    sendPassengerSafe()
    print("\nPASSENGER SAFE")

# Method called at the end of the trip to ensure the passenger exits the vehicle safely.
def passenger_exit():
    exit = 0
    while exit <= 30*4:
        ret_val, final_image = cam.read()


        final_image = edit_video(final_image)

        ###############
        # Process openpose
        ############
        # Process image
        datum = op.Datum()
        datum.cvInputData = final_image
        opWrapper.emplaceAndPop([datum])

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
     
    f.write("Trip ended successfully.\n")
    f.write("Trip time: {} sec\n".format(round((cur_time - start_time), 2)))    
    sendPassengerExit()


# Edit video frame for proper processing.
def edit_video(img):
    # Edit video
    h,w = img.shape[:2]
    cropped = img[:, :int(w/2)]
    final_image = cv2.flip(cropped, -1)
    return final_image

# Method called when passenger is unsafe.
# Sends signal to monitoring/UI
def unsafe():
    sendPassengerUnsafe()
    print("PASSENGER UNSAFE")

###################
# Safety Monitoring
################

# Start networking client
#client.loop_forever()

initial_safety()
#safety_analysis()
#passenger_exit()

