FROM ros:foxy-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update
# Basic tools
RUN apt install -y vim iputils-ping screen iproute2 x11-apps build-essential git
# Configure packages
RUN echo "defshell -bash" >> /etc/screenrc
# Install opencv
RUN apt install -y \
  libopencv-dev\
  python3-opencv
# Install ros utils
RUN apt install -y                  \
  python3-colcon-common-extensions  \
  ros-foxy-rclcpp-action            \
  ros-foxy-ros-testing              \
  ros-foxy-tf2-tools                \
  ros-foxy-tf-transformations       \
  ros-foxy-tf2-geometry-msgs        \
  ros-foxy-behaviortree-cpp-v3      

# I am Groot
RUN apt install -y  \
  qtbase5-dev       \
  libqt5svg5-dev    \
  libzmq3-dev       \
  libdw-dev         \
  libncurses-dev

RUN source /ros_entrypoint.sh
RUN git clone https://github.com/BehaviorTree/Groot.git
RUN mkdir -p /Groot/build
WORKDIR Groot
RUN git submodule update --init --recursive
WORKDIR build
RUN cmake ..
RUN make
RUN make install
WORKDIR ..
RUN ldconfig
RUN mv /Groot /usr/bin/groot
WORKDIR /
RUN rm -rvf /Groot


# Display configuration
ENV DISPLAY=:0
RUN apt install -y x11-apps
