#!/bin/bash

echo "Running Different Velodyne Setup Script"

echo "Shutting Down WiFi"

nmcli radio wifi off

sudo ifconfig eth0 up

echo "Assigning IP to Port"

sudo ip addr add 192.168.1.200 dev eth0

echo "Adding static route to LIDAR IP"

sudo route add -net 192.168.1.0 netmask 255.255.255.0 dev eth0

echo "Try running the velodyne manager now"
