#!/bin/bash
rmdir /media/pendrive

# Export HOME as is needed for ros logs
export HOME=/home/root/

# Load ROS2
source /opt/ros/foxy/setup.bash

# Load nodes
ros2 service call /pendrive_status std_srvs/srv/SetBool "{data: false}"
