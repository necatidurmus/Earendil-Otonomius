FROM osrf/ros:humble-desktop-full

# Env setup
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install general tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    nano \
    python3-pip \
    terminator \
    vim \
    wget \
    x11-apps \
    ros-humble-ros-gz \
    mesa-utils \
    locales \
    # Navigation & SLAM packages
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# Add non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup ROS environment
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# User Setup
USER $USERNAME
WORKDIR /home/$USERNAME/ws

# Source ROS setup in .bashrc options
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
