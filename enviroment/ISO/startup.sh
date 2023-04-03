#!/bin/bash

# Load ROS2
source /opt/ros/foxy/setup.bash
source  ~/krakenvolution/ROS/install

# Load nodes
ros2 launch uahrk_bring_up roboot.py
