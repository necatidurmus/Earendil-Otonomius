FROM ros:humble

# Env setup
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install navigation & sensor fusion packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-pip \
    locales \
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
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS environment
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Create workspace directory with correct ownership
RUN mkdir -p /home/$USERNAME/ws && chown $USERNAME:$USERNAME /home/$USERNAME/ws

# User Setup
USER $USERNAME
WORKDIR /home/$USERNAME/ws

# Source ROS + workspace setup in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "test -f /home/$USERNAME/ws/install/setup.bash && source /home/$USERNAME/ws/install/setup.bash" >> /home/$USERNAME/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
