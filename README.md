# Moon Rover Simulation - ROS2 Docker Environment

Docker environment for testing a rover with lidar navigation on a moon surface.

## Quick Start

### Build the image
```bash
docker build -t moon-rover-sim .
```

### Run the container
```bash
docker run -it --rm \
  --security-opt seccomp=unconfined \
  -p 6080:80 \
  -p 5900:5900 \
  --name moon-rover \
  moon-rover-sim
```

### Access
- **Web VNC**: http://localhost:6080
- **VNC Client**: localhost:5900

**Note**: The `--security-opt seccomp=unconfined` flag is required for Ubuntu Jammy-based images.

## Usage

Once inside the container (via VNC):

### Terminal 1 - Start Simulation
```bash
cd ~/Desktop/ros2_ws
source install/setup.bash
ros2 launch moon_rover_sim simulation.launch.py
```

### Terminal 2 - Start Navigation
```bash
cd ~/Desktop/ros2_ws
source install/setup.bash
ros2 launch moon_rover_sim navigation.launch.py
```

### Navigate
1. In RViz2, click **"2D Goal Pose"**
2. Click and drag on the map to set goal
3. Robot navigates autonomously!

## Features
- 4-wheeled rover with differential drive
- 360° lidar sensor
- Moon environment (low gravity, craters)
- Nav2 autonomous navigation
- SLAM mapping

## Repository
https://github.com/trakyalilter/ros
