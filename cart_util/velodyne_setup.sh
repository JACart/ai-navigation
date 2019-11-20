#!/bin/bash

echo "Running Belodyne Setup Script"

echo "Shutting Down WiFi"

nmcli radio wifi off

echo "Assigning IP to Port"

sudo ifconfig eno1 192.168.3.100

echo "Adding static route to LIDAR IP"

sudo route add 192.168.1.201 eno1

echo "Try running the velodyne manager now"
