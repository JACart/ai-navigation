## Passenger Pose Code

**Author: Software Team S23**

### Summary
This folder contains the main ROS node used for passenger detection the JAcart vehicle, while also holding a working machine learning implementation 
that detects if a passenger is safe or unsafe. There are currently two implementations of this.
- non-ML implementation: passenger.py
- ML implementation: passenger_ML.py

### How to Run
launch the **passenger.py** or **passenger_ML.py** ROS node for passenger pose tracking and object detection
launch the **passenger-ML-data-collect.py** ROS node to collect data for the RandomForestClassifier.


### How to Record through ZED Cameras:
1. go to av_record repository
2. `roslaunch av_record av_record.launch subject:=subject recording_path:=/home/jacart/catkin_ws/src/`
3. from catkin_ws/src, `python ./av_record/scripts/combine.py subject.bag subject.wav`
4. make sure the TTS (microphone) node is off in startcart.
