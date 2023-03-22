## Passenger Pose Code

**Author: Software Team S23**

### Summary
This folder contains the main ROS node used for passenger detection the JAcart vehicle, while also holding a working machine learning implementation 
that detects if a passenger is safe or unsafe. The model itself is a RandomForestClassifier and has gone through a ton of data and model analysis. 

### How to Run
launch the **passenger.py** ROS node for passenger pose tracking and object detection
launch the **passenger-ML-data-collect.py** ROS node to collect data for the RandomForestClassifier.
