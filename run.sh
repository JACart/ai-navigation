#!/bin/bash

# Check for fake pose flag
fake_pose_flag=''

print_usage() {
  printf "Usage: -p activates fake_pose\n"
}

while getopts 'p' flag; do
  case "${flag}" in
    p) fake_pose_flag='true' ;;
    *) print_usage
       exit 1 ;;
  esac
done

# Start everything 

sudo modprobe -r uvcvideo
#echo "Configuring Velodyne..."
#./velodyne_setup.sh
echo "Setting up display"
xinput map-to-output "G2Touch Multi-Touch by G2TSP" HDMI-0
wait
sleep 2
echo "Launching Navigation Code..."
gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch obstacle_detection:=true; exec bash"'
sleep 5
# echo "Starting pose tracking server..."
# gnome-terminal --tab -e 'sh -c "cd ~; cd Desktop/pose-tracking; npm start; exec bash"'
sleep 5
echo "Starting local server..."
# gnome-terminal --tab -e 'sh -c "cd ~; cd Desktop/jakart-local-server; HTTPS=true npm start; exec bash"'
echo "Starting UI"
# gnome-terminal --tab -e 'sh -c "cd ~; cd Desktop/jakart-cart-ui; HTTPS=true npm start; exec bash"'

# Pose tracking
if [ -z "$fake_pose_flag" ]
then
    # launch pose
    echo "Starting pose tracking"
    gnome-terminal --tab -e 'roslaunch cart_endpoints zed.launch'
    sleep 5
    gnome-terminal --tab -e 'python ../jacart-zed/pose/pose_tracking.py'

else
    # launch fake pose
    echo "Starting fake pose tracking"
    gnome-terminal --tab -e 'python ../jacart-zed/pose/fake_pose_tracking.py'
fi
