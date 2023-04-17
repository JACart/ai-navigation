#!/bin/bash

# Check for fake pose flag
pose_flag=''
online_flag=''
full_map=''
full_flag=''
passenger_flag=''

print_usage() {
  printf "Usage: -p activates pose, -o for online, -f for full map\n"
}

while getopts ':fopra' flag; do
  case "${flag}" in
    p) pose_flag='pose' 
    ;;
    o) online_flag='online' 
    ;;
    f) 
       full_map='true'
       full_flag='fullmap'
    ;;
    r)
       research_flag='research'
    ;;
    a) passenger_flag='true'
    ;;
    \?) print_usage
       exit 1 ;;
  esac
done

# Start everything 

#sudo modprobe -r uvcvideo
#echo "Configuring Velodyne..."
#./velodyne_setup.sh

# echo "Setting up display"
# xinput map-to-output "G2Touch Multi-Touch by G2TSP" HDMI-0 -  Not using anymore (03/22)
wait

# zed camera launch
if [ -n "$passenger_flag" ]
then
  echo "Launching ONLY Zed passenger camera node"
  # gnome-terminal --tab -e 'sh -c "cd ~; roslaunch cart_endpoints jacart_passenger_cam.launch; exec bash"'
  gnome-terminal --tab -e 'sh -c "cd ~; roslaunch cart_endpoints jacart_passenger_cam.launch; exec bash"'
  sleep 4
else
  echo "Launching Zed camera nodes"
  gnome-terminal --tab -e 'sh -c "cd ~; roslaunch cart_endpoints jacart_multi_cam.launch; exec bash"'
  sleep 4
fi
#end zed cam stuff.

sleep 2
echo "Launching Navigation Code..."
if [ -n "$full_map" ]
then
  gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch obstacle_detection:=true map_arg:=/home/jacart/AVData/final_map_condensed_5-22.pcd; exec bash"'
else 
  gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch obstacle_detection:=true map_arg:=/home/jacart/AVData/speedBoiMap.pcd; exec bash"'
fi

sleep 4
echo "Starting local server..."
echo 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag $full_flag; exec bash"'
sleep 2
gnome-terminal --tab -e "sh -c \"cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag $full_flag $research_flag; exec bash\""
echo "Starting UI"
gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/cart-ui-offline; npm start; exec bash"'
echo "Starting TTS/STT"
#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/offline-speech-recognition; python3 stt.py; exec bash"'
#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/offline-speech-recognition; python3 tts.py; exec bash"'

# Pose tracking
if [ -n "$pose_flag" ]
then
    # launch pose
    echo "Shim"
    # echo "Starting pose tracking"
    # gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch; exec bash"'
#else
#    # launch fake pose
#    echo "Starting fake pose tracking"
#    gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch fake:=true; exec bash"'
#
fi




