#!/bin/bash

echo "Running Velodyne Setup Script"

echo "Shutting Down WiFi"

nmcli radio wifi off

echo "Assigning IP to Port"

sudo ifconfig enp0s31f6 192.168.3.100

echo "Adding static route to LIDAR IP"

sudo route add 192.168.1.201 enp0s31f6

echo "Try running the velodyne manager now"
