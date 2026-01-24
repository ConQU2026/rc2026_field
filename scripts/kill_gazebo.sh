#!/bin/bash
echo "Stopping ROS 2 daemon..."
ros2 daemon stop

echo "Killing gzserver and gzclient..."
killall -9 gzserver gzclient 2>/dev/null
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f gazebo 2>/dev/null

echo "Killing any lingering ROS launch processes..."
pkill -9 -f rc2026_field_sim.launch.py 2>/dev/null

echo "Gazebo cleanup complete."
