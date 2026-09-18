ARG ROS_DISTRO=kilted
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_INSTALL_PATH=/opt/ros/$ROS_DISTRO

# Install essential development tools, compilers, and ROS 2 build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    gdb \
    git \
    nano \
    net-tools \
    python3-colcon-clean \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-argcomplete \
    sudo \
    udev \
    vim \
    wget \
    mesa-utils \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache \
    mold \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gtsam \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rtabmap \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-rtabmap-viz \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    && rm -rf /var/lib/apt/lists/*

# Create or reuse a non-root user with sudo privileges and hardware group access
RUN if id -u ${USER_UID} >/dev/null 2>&1; then \
        EXISTING_USER=$(id -un ${USER_UID}); \
        if [ "$EXISTING_USER" != "$USERNAME" ]; then \
            usermod -l ${USERNAME} ${EXISTING_USER}; \
            groupmod -n ${USERNAME} $(id -gn ${USERNAME}) || true; \
            usermod -d /home/${USERNAME} -m ${USERNAME}; \
        fi; \
    else \
        groupadd --gid ${USER_GID} ${USERNAME} && \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    fi \
    && usermod -aG sudo,dialout,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Switch to non-root user and configure user environment
USER ${USERNAME}

ENV DEBIAN_FRONTEND=dialog

WORKDIR /workspaces/glidar_slam_demo
