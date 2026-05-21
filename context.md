# Green Guardian – FYP Context File

## Project Overview

Autonomous garbage collection robot that detects and classifies waste
(organic / non-organic) and autonomously collects it using a 6DOF gripper.
Built with ROS2 Jazzy, deployed on Raspberry Pi 5.

## Team

- Mohammed Farhat
- Mohammad Shaaban
- Zakariya Al-Khatib

---

## Hardware

### Computing & Control

- Raspberry Pi 5 (main brain) – Ubuntu 24.04 Server, with active cooler
- Arduino Uno R3 #1 – drive/motor control
- Arduino Uno R3 #2 – 6DOF gripper servo control
- Laptop – Ubuntu 24.04 Desktop (RViz2, development, Node.js backend, testing)

### Sensors

- LiDAR: Slamtec RPLidar A1
- Camera: Logitech C920 1080p
- IMU: GY-91 (accelerometer + gyroscope + magnetometer)
- Ultrasonic sensor – distance measurement for gripper approach

### Actuation

- Motors: 4x JGA 25-370 DC motors with encoders
- Motor driver: DBH-1B (dual channel, controls 4 motors)
- Gripper: 6DOF metal chassis, 6x MG966R servos

### Power

- 5V battery – powers Pi 5 and logic
- 12V battery – powers motors and servos
- Battery voltage monitoring: ADS1115 ADC over I2C (voltage divider on 12V line)
  - R1: 10kΩ, R2: 3.3kΩ → divider scale: 13300/3300 = 4.03
  - ADS1115 wired to Pi 5 I2C (SDA: GPIO2, SCL: GPIO3)
  - Library: adafruit-circuitpython-ads1x15

### Chassis

- 2x plexiglass plates (40cm L x 30cm W x 4mm H)
- Plates separated by standoffs/spacers (2-level design)
- 4x wheels, 75mm diameter, mounted under lower plate via brackets
- 2 bins (front-left: organic, front-right: non-organic)

### Physical Layout

**Upper plate:** LiDAR (top-left, highest point), 6DOF gripper arm
(top-middle/front), Raspberry Pi 5 (back), Arduino #2 gripper (back),
GY-91 IMU (back)

**Lower plate:** 4x JGA motors + wheels (underside), DBH-1B motor
driver, Arduino #1 motor control

**Front face:** Logitech C920 camera (center), 2 bins (left + right)

### Dimensions

- Plate: 40cm x 30cm x 4mm
- Wheel diameter: 75mm (radius: 37.5mm)
- Wheel width: 28.5mm
- Wheel separation (track width): 341mm (center-to-center, left to right)
- Wheelbase: 290mm (center-to-center, front to rear)
- Inter-plate height (standoff length): 58mm
- Ground clearance: 53mm (ground to bottom plate)
- Total chassis height: 66mm (bottom plate + standoffs + top plate)
- Camera mount height from ground: 150mm (lens center)
- Camera pitch: 20° downward
- LiDAR mount height from ground: 279mm (sensor center)
- LiDAR position: +99mm forward, +106mm left of chassis center
- LiDAR standoff height: 145mm (above chassis top)
- Gripper base position relative to chassis center: not modeled in URDF

---

## Software Stack

- ROS2 Jazzy (Ubuntu 24.04)
- Python – ROS2 nodes
- C++ / XACRO – URDF robot description
- SLAM Toolbox – mapping
- AMCL – localization
- Nav2 – autonomous navigation + obstacle avoidance
- YOLOv11 (PyTorch) – waste detection and classification
- OpenCV – image processing
- RViz2 – visualization (runs on laptop)
- Arduino Serial Bridge – low-level motor + gripper control

---

## Web Dashboard

### Stack

- **Frontend:** HTML/CSS/JS — served by Node.js/Express
- **Backend:** Node.js/Express — runs on laptop
- **Database:** MongoDB — stores sensor logs every 10 seconds
- **Real-time:** Socket.IO — backend pushes live data to frontend
- **ROS2 Bridge:** roslib (npm) — backend subscribes to Pi topics via rosbridge WebSocket`
- **Map/RViz View:** noVNC — streams RViz2 from laptop display into dashboard iframe

### Pi Network Info

- Pi IP: `192.168.0.94`
- rosbridge WebSocket: `ws://192.168.0.94:9090`
- Camera stream: `http://192.168.0.94:8080`

### Architecture

```
Pi (ROS2 topics)
    → rosbridge_websocket :9090
        → Node.js backend (roslib subscriber)
            → MongoDB (every 10s)
            → Socket.IO → Frontend (live)

Frontend button → HTTP POST → Node.js → rosbridge service call → Pi
```

### ROS2 Topics Consumed by Dashboard

| Topic | Type | Display |
|---|---|---|
| `/system/cpu_temp` | `std_msgs/Float32` | CPU temperature gauge + chart |
| `/system/cpu_usage` | `std_msgs/Float32` | CPU usage % |
| `/system/ram_usage` | `std_msgs/Float32` | RAM usage % |
| `/system/fan_speed` | `std_msgs/Float32` | Fan speed % |
| `/bins/organic_level` | `std_msgs/Float32` | Organic bin fill % |
| `/bins/non_organic_level` | `std_msgs/Float32` | Non-organic bin fill % |
| `/odom` | `nav_msgs/Odometry` | Robot speed (computed √(vx²+vy²)) |

### ROS2 Services Called by Dashboard

| Button | Service | Type |
|---|---|---|
| Empty Bins | `/bins/reset` | `std_srvs/Trigger` |
| Turn Off Robot | `/robot/shutdown` | `std_srvs/Trigger` |

### Dashboard Pages / Sections

1. **Overview** — temperature gauge, current metrics (temp, speed, bin levels, last update), speed over time chart
2. **Bins** — organic/non-organic fill bars, Empty Bins button
3. **System** — CPU temp chart over time, CPU usage, RAM, fan speed, battery level bar
4. **Tracking** —RViz2 embedded via noVNC iframe

### Tracking Page — noVNC Setup

RViz2 runs natively on the laptop. noVNC captures the laptop display and
serves it into the dashboard iframe. No Pi involvement — entirely localhost.

### ROS2 Workspaces on Pi

- `~/ros2_ws` — original LiDAR test workspace, do not modify
- `~/green_guardian_ws` — all Green Guardian nodes, kept separate

### Key Decisions

- rosbridge chosen over MQTT — native ROS2 service support, local network only
- Node.js backend is the only connection point to Pi — frontend never talks to Pi directly
- Sensor data buffered and written to MongoDB every 10s — not on every message
- GPS/Leaflet map replaced with RViz2 via noVNC iframe
- Humidity removed — no sensor available
- Battery reads from ADS1115 ADC over I2C (stub 0.0 until hardware wired)
- YOLOv11 runs on Pi CPU only — use Nano model, skip frames, run in separate thread
- Node.js + MongoDB run on laptop — Pi is never a web server

---

## ML Model

- Task: Waste detection + classification
- Classes: organic, non-organic (2 classes)
- Dataset: Custom – photos of trash taken at university + background images
- Annotation tool: Label Studio (YOLO format, bounding boxes)
- Dataset size: 400 images
- Models trained: YOLOv11 Nano, YOLOv11 Medium
- Best mAP: 90%
- Deployed model: YOLOv11 Nano preferred (better FPS on Pi 5 CPU)
- Inference: run every 3rd frame, in a separate thread, never blocking ROS2 executor

---

## Robot Behavior (State Machine)

### Phase 1 – Mapping

- Robot explores and builds full map using SLAM Toolbox
- Costmap generated, live visualization on RViz2 via laptop

### Phase 2 – Autonomous Garbage Collection Loop

1. Navigate and explore map using Nav2
2. Camera + YOLOv11 continuously scan for waste
3. If waste detected:
   - Approach using bounding box pixel coordinates for direction
   - Use ultrasonic sensor for distance measurement
   - Stop at target distance
   - Gripper attempts collection (max 3 attempts)
   - If 3 attempts fail → mark location on map, move on
4. Classify waste (organic / non-organic) → deposit in correct bin
5. Repeat exploration loop

### Phase 3 – Full Bin

- Robot navigates to predefined dump point on the map and stops

---

## Arduino #1 Pin Map (Motor Control)

| Pin | Function |
|---|---|
| 2 | Right encoder A (interrupt) |
| 3 | Left encoder A (interrupt) |
| 4 | Left encoder B |
| 5 | IN1B – left forward PWM |
| 6 | IN2A – right reverse PWM |
| 7 | ENA – right enable |
| 8 | ENB – left enable |
| 9 | IN2B – left reverse PWM |
| 10 | IN1A – right forward PWM |
| 12 | Right encoder B |

- Serial protocol: `M:LEFT,RIGHT\n` (commands), `O:LEFT,RIGHT\n` (odometry at 20Hz)
- Ticks per revolution: 293 (empirically measured)
- Right encoder ISR direction software-flipped to correct physical mirroring

---

## Project Status

### Done ✅

- Hardware assembled (partially)
- YOLOv11 model trained (90% mAP, organic/non-organic)
- Custom dataset collected and annotated (400 images)
- Full URDF/XACRO robot description (visualized in RViz2)
- Motor hardware layer: direction test, encoder wiring, ticks/rev calibrated
- `motor_control.ino` and `motor_bridge.py` ROS2 node complete
- Pi-side dashboard infrastructure: `system_monitor_node`, `rosbridge_websocket` running
- All `/system/` topics publishing live on Pi
- Web dashboard (frontend + backend) built with all sections

### In Progress 🔄

- Connecting Node.js backend to Pi via rosbridge
- ADS1115 battery voltage monitoring (hardware not yet wired)

### Not Started ❌

- IMU node
- SLAM configuration
- Nav2 navigation stack
- YOLO perception pipeline (ROS2 node)
- Gripper FSM (Arduino #2)
- Full robot state machine
- web_video_server setup on Pi
- noVNC setup on laptop

### Blockers / Open Questions ⚠️

- Nano vs Medium YOLO model — final decision pending Pi 5 performance test
- Gripper approach strategy (pixel-based direction + ultrasonic depth)
- 3-attempt failure handling and map marking strategy
- ADS1115 wiring and calibration pending

---

## Key Design Decisions

- Two Arduinos: separation of concerns (drive vs gripper)
- Serial bridge between Pi 5 and Arduinos for ROS2 integration
- Bounding box coordinates used for approach alignment
- Ultrasonic sensor used for final approach distance (not LiDAR)
- Max 3 gripper attempts before abandoning object
- Two physical bins on robot for on-board sorting
- Gripper excluded from URDF and RViz visualization
- Gripper operates purely in real life via Arduino #2
- ROS2 sends pick command, waits for completion signal
- Robot appears stationary in RViz during collection
- Two-plate chassis modeled as single box in URDF for simplicity
- Ultrasonic sensors intentionally excluded from URDF

---

## Rules

- Always explain what the code does and why before giving it
- Flag anything that needs to be tested or verified on hardware
- If multiple approaches exist, list tradeoffs before implementing
- Never assume hardware works — always include test steps
- Keep Pi 5 performance in mind (avoid heavy compute on main thread)
- Pin maps and hardware details from Mohammed are ground truth — never infer
- Always give full corrected code files, not partial snippets or diffs
