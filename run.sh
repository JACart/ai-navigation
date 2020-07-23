#!/bin/bash
echo "Configuring Velodyne..."
./velodyne_setup.sh
wait
sleep 2
echo "Starting local server..."
gnome-terminal --tab -e 'sh -c "cd ~; cd Desktop/jackart-local-server; npm start; exec bash"'
echo "Launching Navigation Code..."
sleep 2
gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch; exec bash"'

