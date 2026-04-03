# syntax=docker/dockerfile:1
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Desktop / VNC settings
ENV DISPLAY=:1
ENV VNC_PORT=5901
ENV NOVNC_PORT=6080
ENV VNC_RESOLUTION=1600x900
ENV VNC_COL_DEPTH=24

# ROS defaults
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=30
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Base tools + ROS deps + GUI stack
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    libboost-dev \
    libboost-thread-dev \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    curl \
    wget \
    net-tools \
    x11-xserver-utils \
    dbus-x11 \
    xauth \
    xvfb \
    x11vnc \
    websockify \
    novnc \
    xfce4 \
    xfce4-terminal \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    ros-${ROS_DISTRO}-rviz2 \
    gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    "ros-${ROS_DISTRO}-turtlebot3*" \
    ros-${ROS_DISTRO}-turtlebot3-gazebo \
  && rm -rf /var/lib/apt/lists/*

# Install clearpath simulator for Husky / Jackal simulation
RUN apt-get update && apt-get install -y wget && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y ignition-fortress ros-${ROS_DISTRO}-clearpath-simulator && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-humble-clearpath-simulator

RUN rosdep init || true && rosdep update

WORKDIR /ws
COPY . /ws/src/m-explore-ros2

# Install workspace deps + build
RUN apt-get update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

# noVNC helper symlink commonly used by launch scripts
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

COPY ./entrypoint.sh /entrypoint.sh
COPY ./start_desktop.sh /start_desktop.sh
RUN chmod +x /entrypoint.sh /start_desktop.sh

EXPOSE 5901 6080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start_desktop.sh"]