ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Base utilities and ROS build tools 
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Prepare for Fortress repository 
RUN apt-get update && apt-get install -y lsb-release gnupg

#Step 2: Add OSRF key and repository 
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Ignition Gazebo Fortress and ROS 2 integration
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-ign-ros2-control \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*
# Set rendering engine
ENV IGN_RENDER_ENGINE=ogre2

# Note: User creation is handled by the common-utils feature

# ROS environment 
SHELL ["/bin/bash", "-lc"]
ENV ROS_DISTRO=${ROS_DISTRO}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Workspace layout 
WORKDIR /workspaces/ros2-ws
RUN mkdir -p src

# Entrypoint 
COPY .devcontainer/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
