#!/bin/bash
# Navigation Viewer Launch Script
#
# Usage:
#   ./run.sh           - Network mode (communicate with Docker from host)
#   ./run.sh --local   - Local mode (run inside the same container as ROS2)

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace if available
if [ -f "/home/ubuntu/Desktop/ros2_ws/install/setup.bash" ]; then
    source /home/ubuntu/Desktop/ros2_ws/install/setup.bash
fi

# Set ROS domain ID (must match the Docker container)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Check for --local flag
if [[ "$1" == "--local" ]] || [[ "$NAV_VIEWER_LOCAL" == "1" ]]; then
    echo "🏠 Running in LOCAL mode (same container as ROS2)"
    export ROS_LOCALHOST_ONLY=1
    export NAV_VIEWER_MODE="local"
else
    echo "🌐 Running in NETWORK mode (cross-container communication)"
    export ROS_LOCALHOST_ONLY=0
    export NAV_VIEWER_MODE="network"
fi

echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Run the application
cd "$SCRIPT_DIR"
python3 -m nav_viewer.main "$@"
