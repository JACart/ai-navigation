#!/bin/bash

# Check for fake pose flag
pose_flag=''
online_flag=''
server=''
app=''
front_flag=''

print_usage() {
  printf "Usage: -p activates pose, -o for online, -n disables the npm servers, -a enables the app expo server\n"
}

while getopts ':nopaf' flag; do
  case "${flag}" in
    p) pose_flag='pose' 
    ;;
    o) online_flag='online' 
    ;;
    n) server='noserv'
    ;;
    a) app='enable'
    ;;
    f) front_flag='enable'
    ;;
    \?) print_usage
       exit 1 ;;
  esac
done

# Start everything 

#sudo modprobe -r uvcvideo
#echo "Configuring Velodyne..."
#./velodyne_setup.sh

echo "Launching Navigation Code..."
gnome-terminal --tab -e 'sh -c "roslaunch cart_control navigation.launch obstacle_detection:=true; exec bash"'


if [ -n "$server" ] 
then
	break
else
	sleep 4
	echo "Starting local server..."
	#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start pose online; exec bash"'
	#gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start pose; exec bash"'
	echo 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag; exec bash"'
	gnome-terminal --tab -e "sh -c \"cd ~; cd /home/jacart/catkin_ws/src/local-server; npm start $pose_flag $online_flag; exec bash\""
	echo "Starting UI"
	gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/cart-ui-offline; npm start; exec bash"'
fi



#Pose tracking
if [ -n "$pose_flag" ]
then
    # launch pose
    echo "Starting pose tracking"
    gnome-terminal --tab -e 'sh -c "roslaunch --wait zed_wrapper zed2i.launch serial_number:=31061594 node_name:=passenger_zed camera_name:=passenger_cam; exec bash"'
    gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch; exec bash"'
#else
    # launch fake pose
#    echo "Starting fake pose tracking"
#    gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch fake:=true; exec bash"'

fi




if [ -n "$front_flag" ]
then
	echo "Starting front facing Zed"
	gnome-terminal --tab -e 'sh -c "roslaunch --wait zed_wrapper zed2i.launch serial_number:=37963597 node_name:=front_zed camera_name:=front_cam; exec bash"'
	#gnome-terminal --tab -e 'sh -c "roslaunch --wait jacart-zed pose.launch; exec bash"'

fi


if [ -n "$app" ]
then
	gnome-terminal --tab -e 'sh -c "cd ~; cd /home/jacart/catkin_ws/src/summon-app-rn; expo start; exec bash"'
fi
