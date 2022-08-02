#!/bin/bash

# Load ROS2
source /opt/ros/foxy/setup.bash
source  /ros_ws/install/setup.bash

# Load nodes
ros2 launch bring_up_pkg roboot.launch.py
