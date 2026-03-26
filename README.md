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
- **Perception:** YOLO + C920 Camera
- **Control:** Raspberry Pi 5
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
