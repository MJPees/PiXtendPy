#!/bin/sh
if [ ! -d "/sys/class/gpio/gpio24" ]; then
 echo "24" > /sys/class/gpio/export
fi
echo "out" > /sys/class/gpio/gpio24/direction
echo "1" > /sys/class/gpio/gpio24/value
if [ ! -d "/sys/class/gpio/gpio27" ]; then
 echo "27" > /sys/class/gpio/export
fi
echo "out" > /sys/class/gpio/gpio27/direction
echo "1" > /sys/class/gpio/gpio27/value
sleep 1
sudo modprobe mcp251x
sudo /sbin/ip link set can0 up type can bitrate 125000 restart-ms 100
sudo ip -s -d link show can0

