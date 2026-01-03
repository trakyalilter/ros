# ROS2 Navigation Viewer

A modern PyQt5 application for visualizing ROS2 navigation data, built with **hexagonal architecture** (ports/adapters pattern).

![Python](https://img.shields.io/badge/Python-3.8+-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble-brightgreen)
![PyQt5](https://img.shields.io/badge/PyQt5-5.15+-orange)

## Features

- **Map Visualization**: Real-time occupancy grid display with pan/zoom
- **Laser Scan Overlay**: Live lidar point visualization
- **Navigation Paths**: Global and local path rendering
- **Costmap Display**: Global and local costmap overlays
- **Interactive Navigation**: 
  - Right-click + drag to set navigation goal with orientation
  - Double-click to set initial pose estimate
- **Modern UI**: Dark theme with glassmorphism effects and vibrant accents

## Architecture

```
nav_viewer/
├── domain/          # Core business logic (entities, services)
├── ports/           # Abstract interfaces (input/output)
├── adapters/        # ROS2 implementations
├── infrastructure/  # PyQt5 UI layer
└── application/     # Dependency injection & coordination
```

## Prerequisites

- ROS2 Humble (or compatible)
- Python 3.8+
- PyQt5

## Installation

```bash
cd /home/ubuntu/Desktop/ros2_ws/src/nav_viewer_app

# Install Python dependencies
pip install -r requirements.txt

# Make run script executable
chmod +x run.sh
```

## Usage

### Running with ROS2 Simulation

1. **Start the simulation in Docker:**
   ```bash
   # Inside Docker container
   ros2 launch moon_rover_sim simulation.launch.py
   ros2 launch moon_rover_sim navigation.launch.py
   ```

2. **Run the viewer (same machine or networked):**
   ```bash
   # Ensure same ROS_DOMAIN_ID as the container
   export ROS_DOMAIN_ID=0
   
   # Option 1: Use the run script
   ./run.sh
   
   # Option 2: Run directly
   source /opt/ros/humble/setup.bash
   python3 -m nav_viewer.main
   ```

### Cross-Container Communication

If running the viewer outside the Docker container:

1. Ensure both use the same `ROS_DOMAIN_ID` (default: 0)
2. Make sure network allows DDS communication (no firewall blocking)
3. Docker should be running with `--network host` or appropriate port mappings

## Controls

| Action | Description |
|--------|-------------|
| **Right-click + drag** | Set navigation goal (drag direction = orientation) |
| **Double-click** | Set initial pose estimate |
| **Left-click + drag** | Pan the view |
| **Scroll wheel** | Zoom in/out |
| **Home key** | Reset view |
| **Escape** | Cancel navigation |
| **Ctrl+C** | Center on robot |

## Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | SLAM map |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/plan` | `nav_msgs/Path` | Global navigation path |
| `/local_plan` | `nav_msgs/Path` | Local planner path |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global costmap |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local costmap |

## Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Navigation goal |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose estimate |

## License

MIT License
