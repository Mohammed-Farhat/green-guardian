# The Green Guardian 🌱🤖

An autonomous garbage collection robot designed for university campuses and smart environments.

## Project Overview

The Green Guardian is a ROS2-based autonomous robot capable of:

- SLAM-based navigation (LiDAR + AMCL)
- Real-time waste detection and classification (YOLO)
- Autonomous trash collection and segregation
- Solar-assisted sustainable power
- Remote monitoring and control via a web dashboard

## System Architecture

- **Navigation:** SLAM Toolbox, AMCL, Nav2
- **Perception:** YOLO + Intel RealSense depth
- **Control:** Raspberry Pi 5
- **AI Processing:** Jetson Nano
- **Middleware:** ROS 2
- **Backend:** FastAPI + MQTT
- **Frontend:** Web Dashboard

## Repository Structure

- `ros2_ws/` → ROS2 packages
- `ai/` → Datasets and AI models
- `hardware/` → Schematics and power design
- `web/` → Dashboard backend & frontend
- `docs/` → FYP report and diagrams

## Team

Final Year Project – Computer Engineering  
Beirut Arab University

## License

MIT License

## Simulation navigation (Gazebo + SLAM + Nav2)

This repo includes:

- Gazebo robot launch: `ros2 launch robot_gazebo show_robot.launch.py`
- Nav2 + SLAM (mapping): `ros2 launch robot_navigation slam_nav.launch.py`
- Nav2 + AMCL (localization): `ros2 launch robot_navigation localize_nav.launch.py map:=<path/to/map.yaml>`

### 1) Install required ROS 2 packages

Nav2 is not vendored in this repo. Install it on your machine:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-map-server \
  ros-jazzy-nav2-amcl
```

### 2) Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3) SLAM (build a map) then save it

```bash
ros2 launch robot_navigation slam_nav.launch.py
```

Save a map (example):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/arena_map
```

### 4) Localization + autonomous navigation

```bash
ros2 launch robot_navigation localize_nav.launch.py map:=~/arena_map.yaml
```
