#!/bin/bash

# OBSOLETE!  Set up a static IP for the velodyne in 
# the network manager

echo "Running Velodyne Setup Script"

echo "Shutting Down WiFi"

nmcli radio wifi off

echo "Assigning IP to Port"

#sudo ifconfig enx70886b866bd3 192.168.3.100
sudo ifconfig enp0s31f6 192.168.3.100

echo "Adding static route to LIDAR IP"

#sudo route add 192.168.1.201 enx70886b866bd3
sudo route add 192.168.1.201 enp0s31f6

echo "Try running the velodyne manager now"

sleep 5

nmcli radio wifi on
