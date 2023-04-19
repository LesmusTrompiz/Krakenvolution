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

# Install editor
RUN apt install -y            \
  software-properties-common  \
  wget                        \
  unzip                       \
  curl                        \
  npm

  # Prepara node for latest version
RUN apt update
RUN curl -sL https://deb.nodesource.com/setup_18.x | bash - 
RUN apt install -y            \
  nodejs                      

RUN add-apt-repository -y ppa:neovim-ppa/unstable
RUN apt update
RUN apt install -y neovim
RUN git clone https://github.com/AstroNvim/AstroNvim ~/.config/nvim
RUN git clone https://github.com/DiegoMoralesRoman/AstroNVIMConfig ~/.config/nvim/lua/user

  # Install dependencies for editor LSP

# Install python dependencies
RUN apt install -y python3-pip
RUN python3 -m pip install \
  pyserial

# Display configuration
ENV DISPLAY=:0
RUN apt install -y x11-apps
