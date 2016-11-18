#!/bin/sh
sudo /sbin/ip link set can0 down
echo "1" > /sys/class/gpio/gpio24/value
echo "0" > /sys/class/gpio/gpio27/value
sleep 1
sudo modprobe -r mcp251x 
