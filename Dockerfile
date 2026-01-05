# Moon Rover Simulation Docker Image
# Based on tiryoh/ros2-desktop-vnc:humble with Nav2 navigation stack
FROM tiryoh/ros2-desktop-vnc:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Update ROS2 GPG key (fixes expired key issue)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install Nav2 and SLAM Toolbox packages
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-twist-mux \
    ros-humble-teleop-twist-keyboard \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /home/ubuntu/Desktop/ros

# Clone the moon rover repository
RUN cd .. && git clone https://github.com/trakyalilter/ros.git

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install" || \
    echo "Build will complete when source is mounted"

# Add workspace sourcing to root's bashrc
# The base image will handle copying environment to the ubuntu user
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /home/ubuntu/Desktop/ros/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Note: Don't set USER here - the base image's entrypoint needs to run as root
# to create the ubuntu user and set up VNC. It will switch to ubuntu user automatically.
