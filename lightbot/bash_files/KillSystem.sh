#!/bin/bash

# Bash file for stopping the System - Turning Off the system

# Kill all ROS nodes, launch files, roscore, and related processes
echo "Terminating all ROS processes..."
pkill -f roslaunch
pkill -f rosrun
killall -9 robot_state_publisher

# make sure the camera is properly stopped before restarting else it will give error
rosnode kill /camera
killall mono_camera_node

echo "Cleaning up any remaining processes..."
pkill -f "rviz"

echo "Cleaning up python process..."
pkill -f sample_client.py

echo "System is now clean!!!"