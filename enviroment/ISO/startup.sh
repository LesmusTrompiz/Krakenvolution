#!/bin/bash

# Load ROS2
source /opt/ros/foxy/setup.bash
source /home/krakenvolution/Krakenvolution/ROS/install/setup.bash

# Load nodes
ros2 launch uahrk_bring_up roboot.py
