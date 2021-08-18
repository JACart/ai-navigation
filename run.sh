#!/bin/bash

# Check for fake pose flag
pose_flag=''
online_flag=''

print_usage() {
  printf "Usage: -p activates pose, -o for online\n"
}

while getopts ':op' flag; do
  case "${flag}" in
    p) pose_flag='pose' 
    ;;
    o) online_flag='online' 
    ;;
    \?) print_usage
       exit 1 ;;
  esac
done

# Start everything 

#sudo modprobe -r uvcvideo
#echo "Configuring Velodyne..."
#./velodyne_setup.sh
echo "Setting up display"
xinput map-to-output "G2Touch Multi-Touch by G2TSP" HDMI-0
wait
sleep 2
echo "Launching Navigation Code..."
gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch obstacle_detection:=true; exec bash"'
sleep 4
echo "Starting local server..."
#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start pose online; exec bash"'
#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start pose; exec bash"'
echo 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag; exec bash"'
gnome-terminal --tab -e "sh -c \"cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag; exec bash\""
echo "Starting UI"
gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/cart-ui-offline; npm start; exec bash"'

# Pose tracking
if [ -n "$pose_flag" ]
then
    # launch pose
    echo "Starting pose tracking"
    gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch; exec bash"'
#else
#    # launch fake pose
#    echo "Starting fake pose tracking"
#    gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch fake:=true; exec bash"'
#
fi
