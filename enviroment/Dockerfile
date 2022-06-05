FROM ros:foxy-ros-core

RUN apt update
# Basic tools
RUN apt install -y vim iputils-ping screen iproute2 x11-apps build-essential 
# Configure packages
RUN echo "defshell -bash" >> /etc/screenrc
# Install opencv
RUN apt install -y libopencv-dev python3-opencv
# Install ros utils
RUN apt install -y python3-colcon-common-extensions ros-foxy-rclcpp-action 

# Display configuration
ENV DISPLAY=:0
RUN apt install -y x11-apps
