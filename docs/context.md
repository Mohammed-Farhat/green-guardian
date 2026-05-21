# Green Guardian — Project Context

## 1. Project overview

**Green Guardian** is the Final Year Project of a Computer Engineering team at **Beirut Arab University**. It is an autonomous mobile robot whose mission cycle is:

1. **Map** an unknown indoor area (SLAM).
2. **Wander** the saved map while avoiding obstacles.
3. **Detect** garbage on the floor and **classify** it as organic or non-organic.
4. **Approach** the item, **pick it up** with an on-board arm + gripper.
5. **Drop** it into the matching on-board bin (green = organic, yellow = non-organic).

This document describes the system as it actually exists in the repository today. Anything not listed here (vision pipeline, web dashboard, etc.) is either not yet implemented or out of scope.

---

## 2. Compute & networking

| Machine | Role | OS | Address |
|---|---|---|---|
| Raspberry Pi 5 | On-robot ROS 2 host — runs every node | Ubuntu 24.04 Server | `192.168.0.94` |
| Laptop | Dev / RViz / teleop | Ubuntu 24.04 Desktop | DHCP |

Both machines run ROS 2 Jazzy and share a `ROS_DOMAIN_ID` over Wi-Fi.

---

## 3. Hardware inventory

| Component | Detail |
|---|---|
| Drive motors | 4× **JGA25-370** DC motors with quadrature encoders (293 ticks/rev, calibrated). Differential drive — left pair and right pair driven together. |
| Motor driver | **DHB-1 (DBH-1B)**, max PWM 249 (≈ 98 % of 255). |
| Motor MCU | **Arduino Uno R3 #1** — receives PWM commands and reports encoder counts over USB-serial. |
| Arm + gripper | 6-DOF arm with gripper, driven by **Arduino Uno R3 #2** running an **independent finite-state machine**. Not exposed to ROS 2 — see §10. |
| LiDAR | **Slamtec RPLidar A1**, USB-serial. Frame `laser_link`, topic `/scan`. |
| IMU | **GY-91** (the MPU-6500 die is used; BMP-280 / AK8963 are present on the board but unused). I²C bus 1, address `0x68`. |
| Camera | **Logitech C920** USB webcam, mounted on the front face, pitched 20° down. Driver not yet launched in the workspace — frame is reserved in the URDF. |
| Bins | Two on-board boxes attached to the front of the chassis: `bin_left_link` (organic, green), `bin_right_link` (non-organic, yellow). |

---

## 4. ROS 2 stack

ROS 2 **Jazzy** with:

- **slam_toolbox** — async online mapping.
- **Nav2** — `nav2_amcl`, `nav2_planner` (Dijkstra/NavfnPlanner), `nav2_controller` (DWB), `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `nav2_map_server`.
- **robot_localization** — `ekf_filter_node` fuses wheel odometry + IMU → `/odometry/filtered`.
- **robot_state_publisher** + xacro — URDF processing and TF publishing.

---

## 5. Workspace packages (`ros2_ws/src/`)

### `robot_description`
URDF (xacro) and the only place where SLAM + Nav2 are launched today.

- `urdf/robot.urdf.xacro` — top-level, includes `dimensions.xacro`, `materials.xacro`, `base.xacro`, `sensors.xacro`.
- `urdf/dimensions.xacro` — single source of truth for every measurement (see `dimensions.md`).
- `launch/display.launch.py` — RViz preview (no hardware needed).
- `launch/slam.launch.py` — runs `slam_toolbox` (async) with `config/slam_toolbox.yaml`.
- `launch/nav2.launch.py` — brings up `map_server` + `amcl` + controller/planner/behavior/BT servers + lifecycle manager from `config/nav2_params.yaml`. Requires `map:=<path>` arg.

### `robot_motors`
Bridge between ROS 2 and Arduino #1.

- Node: **`motor_bridge`** (`robot_motors/motor_bridge.py`).
  - Subscribes `/cmd_vel`, applies differential-drive kinematics, scales by `max_velocity` to PWM (clamped to ±`max_pwm`), sends `M:LEFT,RIGHT\n` over serial.
  - Background reader thread parses encoder lines `O:LEFT,RIGHT\n` at 20 Hz, integrates pose using midpoint method, publishes `/odom` and broadcasts TF `odom → base_footprint`.
  - 10 Hz watchdog stops the motors if no `/cmd_vel` arrives within `cmd_vel_timeout` (0.5 s).
  - On shutdown sends `M:0,0\n` and closes the serial port.
- Firmware: `arduino/motor_control.ino` (sketch for Arduino #1, see §8).
- Config: `config/hardware_params.yaml`.
- Launch: `launch/motor_bridge.launch.py` with overridable `serial_port` arg.

### `robot_imu`
GY-91 driver and the EKF.

- Node: **`imu_node`** (`robot_imu/imu_node.py`).
  - Reads MPU-6500 over I²C at 50 Hz: 14-byte burst from register `0x3B` → accel (±2 g, 16384 LSB/g), gyro (±250 °/s, 131 LSB/°/s), temperature.
  - Publishes `/imu` (`sensor_msgs/Imu`, `frame_id = imu_link`, orientation covariance `-1` so consumers ignore orientation) and `/imu/temperature` (`std_msgs/Float32`).
- EKF: `ekf_filter_node` from `robot_localization`, parameters in `config/ekf.yaml`.
- Launch: `launch/imu.launch.py` starts both nodes together.

### `rplidar_ros`
Vendored upstream Slamtec driver — **unmodified** (`https://github.com/Slamtec/rplidar_ros.git`, branch `ros2`). Use `rplidar_a1_launch.py`.

---

## 6. TF tree

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── wheel_front_left
            ├── wheel_front_right
            ├── wheel_rear_left
            ├── wheel_rear_right
            ├── imu_link
            ├── lidar_standoff_link
            │   └── laser_link
            ├── camera_link
            │   └── camera_optical_link
            ├── bin_left_link        (green / organic)
            └── bin_right_link       (yellow / non-organic)
```

`map → odom`: published by `slam_toolbox` (mapping) or `nav2_amcl` (localization).
`odom → base_footprint`: published by `motor_bridge` and (when running) the EKF — `robot_localization` takes over once `ekf_filter_node` is up because `publish_tf: true`.
Everything below `base_link` is fixed and published by `robot_state_publisher`.

---

## 7. Topic / interface table

| Topic | Type | Direction | Frame / rate | Notes |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | sub by `motor_bridge` | — | from teleop or Nav2 velocity smoother |
| `/odom` | `nav_msgs/Odometry` | pub by `motor_bridge` | `odom` → `base_footprint`, ~20 Hz | raw wheel odometry |
| `/imu` | `sensor_msgs/Imu` | pub by `imu_node` | `imu_link`, 50 Hz | orientation cov = −1 (unset) |
| `/imu/temperature` | `std_msgs/Float32` | pub by `imu_node` | — | °C |
| `/scan` | `sensor_msgs/LaserScan` | pub by `rplidar_node` | `laser_link` | from RPLidar A1 |
| `/odometry/filtered` | `nav_msgs/Odometry` | pub by `ekf_filter_node` | `odom` → `base_footprint` | consumed by Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | pub by `slam_toolbox` / `map_server` | — | mapping vs. AMCL mode |
| `/joint_states` | `sensor_msgs/JointState` | pub by `joint_state_publisher_gui` (display launch only) | — | manual wheel sliders for RViz preview |

---

## 8. Hardware-side protocols

### Arduino #1 — motor controller (`arduino/motor_control.ino`)

USB-serial @ **115200 baud**. Pin map:

| Arduino pin | Function | Driver pin |
|---|---|---|
| 2 | Right encoder A (INT0) | — |
| 3 | Left encoder A (INT1) | — |
| 4 | Left encoder B | — |
| 5 (PWM) | LEFT forward | IN1B |
| 6 (PWM) | RIGHT reverse | IN2A |
| 7 | RIGHT enable | ENA |
| 8 | LEFT enable | ENB |
| 9 (PWM) | LEFT reverse | IN2B |
| 10 (PWM) | RIGHT forward | IN1A |
| 12 | Right encoder B | — |

Serial frames:

| From → To | Frame | Meaning |
|---|---|---|
| Pi → Arduino | `M:<left>,<right>\n` | PWM in `[-249, 249]`, sign = direction |
| Arduino → Pi | `O:<left>,<right>\n` | Encoder ticks, sent every 50 ms (20 Hz) |
| Arduino → Pi | `GG:ready\n` | Boot confirmation |
| Arduino → Pi | `OK\n` | Command ACK (logged at debug) |
| Arduino → Pi | `E:<msg>\n` | Error |

The right motor's encoder direction is flipped in firmware because the right motor is mounted mirror-imaged. A **500 ms watchdog** on the Arduino itself stops both motors if no `M:` command arrives.

### GY-91 IMU — I²C (`robot_imu/imu_node.py`)

| Register | Address | Use |
|---|---|---|
| `PWR_MGMT_1` | `0x6B` | Cleared to 0 to wake from sleep |
| `ACCEL_CFG` | `0x1C` | `0x00` → ±2 g range |
| `GYRO_CFG` | `0x1B` | `0x00` → ±250 °/s range |
| `ACCEL_XOUT` | `0x3B` | Start of 14-byte burst (accel + temp + gyro) |
| `TEMP_OUT` | `0x41` | T_C = raw / 333.87 + 21.0 |
| `GYRO_XOUT` | `0x43` | — |

Conversions: accel raw / 16384 × 9.80665 → m/s²; gyro raw / 131 × π/180 → rad/s.

---

## 9. Launch reference

```bash
# Source workspace
source /opt/ros/jazzy/setup.bash
cd ~/green-guardian/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 1. URDF preview (laptop only, no hardware)
ros2 launch robot_description display.launch.py

# 2. Bring up real robot drivers (run on Pi5)
ros2 launch robot_motors  motor_bridge.launch.py
ros2 launch robot_imu     imu.launch.py
ros2 launch rplidar_ros   rplidar_a1_launch.py

# 3. Build a map (drive with teleop in another terminal)
ros2 launch robot_description slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. Save the map
ros2 run nav2_map_server map_saver_cli -f ~/arena_map

# 5. Autonomous navigation on a saved map
ros2 launch robot_description nav2.launch.py map:=~/arena_map.yaml
# → in RViz: use "2D Pose Estimate" to seed AMCL, then "Nav2 Goal" to drive
```

---

## 10. Out of ROS 2 scope

- **Arm + gripper (Arduino #2).** Implemented as a self-contained finite state machine on the second Arduino. It is **not** exposed as ROS 2 topics, services, or actions — there is no URDF link, no joint state, no controller. Coordination with the navigation stack (when to stop, when to grab) will happen through whatever side-channel signal the team chooses; this layer is not yet defined in the repo.
- **Vision / classification.** No node, no model, no topic exists yet. The Logitech C920 is mounted and its frames are reserved in the URDF, but no driver is launched.
