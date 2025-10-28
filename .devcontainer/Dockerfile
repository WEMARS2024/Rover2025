# syntax=docker/dockerfile:1

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Locale & basics
ENV DEBIAN_FRONTEND=noninteractive
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
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Non-root user that matches VS Code devcontainer default
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} vscode && useradd -m -s /bin/bash -u ${UID} -g ${GID} vscode

# ROS env
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Workspace layout
WORKDIR /workspaces/ros2-ws
RUN mkdir -p src

# Entrypoint that overlays workspace if built
COPY .devcontainer/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD [ "bash" ]
