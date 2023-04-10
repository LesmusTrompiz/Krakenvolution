#!/bin/bash

# Export HOME as is needed for ros logs
export HOME=/home/root/

# Load ROS2
source /opt/ros/foxy/setup.bash
source /root/Krakenvolution/ROS/install/setup.bash

# Load nodes
ros2 launch uahrk_bring_up roboot.py
