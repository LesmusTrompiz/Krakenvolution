#!/bin/bash

#Install UTF-8
apt update && apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repos
apt install -y software-properties-common
add-apt-repository -y universe

apt update && apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS
echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list
apt update
apt upgrade -y
apt install -y ros-foxy-ros-base

# Base utils
apt install -y vim iputils-ping screen iproute2 screen
apt install -y libopencv-dev python3-opencv
apt install -y python3-colcon-common-extensions ros-foxy-rclcpp-action ros-foxy-ros-testing

# Create ROS WS
mkdir -p ~/ros_ws/src

# Create start service
cp ./roboot.service /etc/systemd/system 
mkdir /etc/eurobot
cp ./startup.sh /etc/eurobot

systemctl daemon-reload
systemctl enable roboot.service
systemctl start roboot.service
