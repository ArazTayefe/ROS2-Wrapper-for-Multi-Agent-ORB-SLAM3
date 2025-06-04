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
- cv_bridge
- ORB-SLAM3 core (must be built separately)

## Installing and Using mocap4r2 as Ground Truth

mocap4r2 is a ROS2 package designed to integrate motion capture systems with ROS2. It provides tools and nodes to interface with motion capture systems, allowing for real-time tracking and data integration in ROS2 applications. This project uses mocap4r2 to integrate OptiTrack Flex 13 cameras.

1. Ensure your motion capture cameras are set up and calibrated correctly.
2. Make sure that the configuration file is correct. The server_address is the IP of the PC that runs the motion capture software, and local_address is the IP address of the PC that runs the mocap4r2.
3. Launch the mocap4r2 nodes to start streaming motion capture data to ROS2:
```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```
4. As the driver node is a lifecycle node, you should transition to activate:
```bash
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
For more information, visit the [mocap4r2](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack) GitHub repository.

## Build and Run Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select orbslam3
source install/setup.bash
ros2 launch orbslam3 all_drones_orbslam3.launch.py
```

## Repository Origin

This repository is based on and originally forked from the excellent work by [ORB_SLAM3_ROS2 by zang09](https://github.com/zang09/ORB_SLAM3_ROS2).
