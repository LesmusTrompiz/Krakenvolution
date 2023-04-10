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

# Copy udev rules 
cp /root/Krakenvolution/enviroment/ISO/10-uahrkudev.rules /etc/udev/rules.d/
cp /root/Krakenvolution/enviroment/ISO/mount_pendrive.sh /usr/local/bin/
cp /root/Krakenvolution/enviroment/ISO/mount_pendrive.service /etc/systemd/system/
cp /root/Krakenvolution/enviroment/ISO/pendrive_disconnect.sh /usr/local/bin/
chmod +x /usr/local/bin/mount_pendrive.sh
chmod +x /usr/local/bin/pendrive_disconnect.sh


# Create start service
cp ./roboot.service /etc/systemd/system 
mkdir /etc/eurobot
cp ./startup.sh /etc/eurobot

systemctl daemon-reload
systemctl enable roboot.service
systemctl start roboot.service

# Enable root user
#echo -e "krakenvolution\nkrakenvolution" | sudo passwd root

# Enables root login in ssh 
sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Enables ssh conection
apt install -y openssh-server
systemctl enable ssh
systemctl start ssh

# Install things
apt install -y kitty
apt install -y python3-pip

# Install extern ROS PACKAGES
apt install ros-$ROS_DISTRO-behaviortree-cpp-v3
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /root/Krakenvolution/ROS/src/


