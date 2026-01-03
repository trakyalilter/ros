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
WORKDIR /home/ubuntu/Desktop/ros2_ws

# Clone the moon rover repository
RUN git clone https://github.com/trakyalilter/ros.git src/moon_rover_sim || \
    (mkdir -p src/moon_rover_sim && echo "Repository clone failed, mount manually")

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install" || \
    echo "Build will complete when source is mounted"

# Set ownership to ubuntu user
RUN chown -R ubuntu:ubuntu /home/ubuntu/Desktop/ros2_ws

# Add workspace sourcing to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/Desktop/ros2_ws/install/setup.bash 2>/dev/null || true" >> /home/ubuntu/.bashrc

# Set default user
USER ubuntu
WORKDIR /home/ubuntu/Desktop/ros2_ws

# Default command (VNC is already set up by base image)
CMD ["/bin/bash"]
