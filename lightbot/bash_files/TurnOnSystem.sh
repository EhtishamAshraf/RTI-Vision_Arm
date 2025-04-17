#!/bin/bash

# Bash file for controlling the System - Turning On the system

# Source commands:
SOURCE_CMD="source /home/esirem/Desktop/M2_Thesis/4-RTI_Bot/eva_rti_ws/devel/setup.bash"

# Launching all the nodes in separate tabs:
# make sure the camera is properly stopped before restarting else it will give error
gnome-terminal --tab --title="Camera" -- bash -c "$SOURCE_CMD; sleep 5; roslaunch avt_vimba_camera mono_camera.launch; exec bash"
gnome-terminal --tab --title="XY Platform Server" -- bash -c "$SOURCE_CMD; sleep 10; rosrun xy_platform_control xy_platform_server.py; exec bash"
gnome-terminal --tab --title="XY Platform Client" -- bash -c "$SOURCE_CMD; sleep 20; rosrun xy_platform_control xy_platform_client.py; bash"
gnome-terminal --tab --title="EVA Moveit" -- bash -c "$SOURCE_CMD; sleep 35; roslaunch eva_without_rail_moveit_config EvaHardware.launch; exec bash"
gnome-terminal --tab --title="EVA Service" -- bash -c "$SOURCE_CMD; sleep 40; rosrun eva_control eva_service.py; exec bash"
gnome-terminal --tab --title="EVA Control" -- bash -c "$SOURCE_CMD; sleep 45; rosrun eva_control eva_ros_control_server.py; exec bash"
gnome-terminal --tab --title="Lightbot" -- bash -c "$SOURCE_CMD; sleep 50; roslaunch lightbot lightbot.launch; exec bash"
gnome-terminal --tab --title="LightBot Client" -- bash -c "sleep 55; python3 /home/esirem/Desktop/M2_Thesis/4-RTI_Bot/eva_rti_ws/src/lightbot/scripts/sample_client.py; bash"

echo "All nodes launched successfully in separate tabs!"