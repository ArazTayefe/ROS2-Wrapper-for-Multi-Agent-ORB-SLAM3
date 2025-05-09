# ROS2 Wrapper for Multi-Agent ORB-SLAM3

This repository contains a ROS2 wrapper for ORB-SLAM3 extended to support multiple monocular agents (e.g., drones) running concurrently. Each agent processes its camera feed independently and publishes its own SLAM data.

## Features

- Multi-agent support for monocular SLAM
- ROS2-compliant launch system
- Image subscription via `/Alpha/image_raw`, `/Bravo/image_raw`, `/Charlie/image_raw`
- Optional Pangolin viewer support
- Saves trajectory to `KeyFrameTrajectory_<agent>.txt` on shutdown


## Dependencies

- ROS2 Humble or newer
- OpenCV ≥ 4
- Pangolin (optional for visualization)
- `cv_bridge`
- ORB-SLAM3 core (must be built separately)

## Build and Run Instructions

```
cd ~/ros2_ws
colcon build --packages-select orbslam3
source install/setup.bash
ros2 launch orbslam3 all_drones_orbslam3.launch.py
```
