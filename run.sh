#!/bin/bash
echo "Configuring Velodyne..."
./velodyne_setup.sh
echo "Launching Navigation Code"
roslaunch cart_control navigation.launch

