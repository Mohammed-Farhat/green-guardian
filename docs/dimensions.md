# Green Guardian — Dimensions & Tuning Reference

All lengths are in **metres**, masses in **kg**, angles in **radians**, inertias in **kg·m²**. Coordinates are URDF-standard (X = forward, Y = left, Z = up). Source files are cited next to each block — the URDF treats `urdf/dimensions.xacro` as the single source of truth, and that file already does the arithmetic; numbers below are the materialised values.

---

## 1. Chassis (`base_link`)

> Source: `urdf/dimensions.xacro` (chassis section), `urdf/base.xacro`

| Property | Value |
|---|---|
| Length (X) | **0.400** m |
| Width (Y) | **0.300** m |
| Height (Z) | **0.066** m  (4 mm bottom plate + 58 mm standoff stack + 4 mm top plate) |
| Ground clearance to chassis bottom | **0.053** m |
| Chassis top above ground | 0.119 m (`chassis_clearance + chassis_height`) |
| `base_link` Z origin above ground | **0.086** m |
| `base_footprint → base_link` joint | fixed at xyz (0, 0, 0.086) |
| Mass | 3.0 kg (chassis + electronics + batteries lumped) |
| Inertia (Ixx, Iyy, Izz) | 0.0226, 0.0334, 0.0525 — cross-terms 0 |

---

## 2. Wheels (×4)

> Source: `urdf/dimensions.xacro` (wheels section), `urdf/base.xacro`

| Property | Value |
|---|---|
| Radius | **0.0375** m (75 mm diameter) |
| Width  | **0.0285** m |
| Track width (centre-to-centre, L↔R) | **0.341** m |
| Wheelbase (centre-to-centre, F↔R) | **0.290** m |
| Mass per wheel | 0.2 kg |
| Inertia per wheel | Ixx = Iyy = 1.48 × 10⁻⁴, Izz = 1.41 × 10⁻⁴ |
| Joint type | continuous, axis (0, 1, 0) |

Joint origins relative to `base_link`:

| Wheel | x (m) | y (m) | z (m) |
|---|---|---|---|
| `wheel_front_left`  | +0.145 | +0.1705 | −0.0485 |
| `wheel_front_right` | +0.145 | −0.1705 | −0.0485 |
| `wheel_rear_left`   | −0.145 | +0.1705 | −0.0485 |
| `wheel_rear_right`  | −0.145 | −0.1705 | −0.0485 |

`wheel_z = wheel_radius − base_link_z = 0.0375 − 0.086 = −0.0485` m (centres sit below the chassis centre).

---

## 3. LiDAR (RPLidar A1 + standoff)

> Source: `urdf/dimensions.xacro` (lidar section), `urdf/sensors.xacro`

**Standoff** (gold cylinder):

| Property | Value |
|---|---|
| Radius | 0.010 m |
| Height | 0.145 m |
| Mass | 0.05 kg |
| `base_link → lidar_standoff_link` | fixed, xyz (0.099, 0.106, 0.1055) |

**Lidar body** (black puck):

| Property | Value |
|---|---|
| Radius | 0.035 m |
| Height | 0.030 m |
| Mass | 0.17 kg (datasheet ≈ 170 g) |
| Inertia (Ixx, Iyy, Izz) | 6 × 10⁻⁵, 6 × 10⁻⁵, 1.041 × 10⁻⁴ |
| `lidar_standoff_link → laser_link` | fixed, xyz (0, 0, 0.0875), rpy (0, 0, π) |

The 180° yaw rotation makes the LiDAR's 0° heading point forward (it is physically mounted facing rear). Effective scan plane is roughly **0.365 m above the ground** (0.053 + 0.066/2 + 0.0725 + 0.0725 + 0.015 ≈ 0.365 m using the full chain).

---

## 4. Camera (Logitech C920)

> Source: `urdf/dimensions.xacro` (camera section), `urdf/sensors.xacro`

| Property | Value |
|---|---|
| Box size (depth × width × height) | 0.045 × 0.094 × 0.029 m |
| Mass | 0.10 kg |
| `base_link → camera_link` | fixed, xyz (0.2225, 0, 0.064), rpy (0, 0.349, 0) |
| Pitch | **0.349 rad ≈ 20° nose-down** |
| `camera_link → camera_optical_link` | fixed, xyz (0, 0, 0), rpy (−π/2, 0, −π/2) — ROS optical convention (Z forward, Y down) |

Camera sits at **0.150 m above the ground** (`base_link_z + camera_z = 0.086 + 0.064`).

---

## 5. IMU (GY-91)

> Source: `urdf/dimensions.xacro` (IMU section), `urdf/base.xacro`

| Property | Value |
|---|---|
| Box size (X × Y × Z) | 0.020 × 0.015 × 0.003 m |
| Mass | 0.01 kg |
| `base_link → imu_link` | fixed, xyz (0, 0, 0.0345), rpy (0, 0, 0) |

Sits centred on the chassis top plate.

---

## 6. Bins

> Source: `urdf/dimensions.xacro` (bins section), `urdf/sensors.xacro`

| Property | Value |
|---|---|
| Box size (depth × width × height) | 0.080 × 0.120 × 0.066 m |
| Mass | 0.15 kg each |
| Inertia (Ixx, Iyy, Izz) | 5 × 10⁻⁵ each |

Joint origins relative to `base_link`:

| Bin | Colour | Purpose | x | y | z |
|---|---|---|---|---|---|
| `bin_left_link`  | green  | organic     | +0.240 | +0.090 | −0.042 |
| `bin_right_link` | yellow | non-organic | +0.240 | −0.090 | −0.042 |

Both extend 0.080 m beyond the front face of the chassis. Their tops sit flush with the chassis top.

---

## 7. Differential-drive parameters

> Source: `robot_motors/config/hardware_params.yaml`

| Parameter | Value |
|---|---|
| `serial_port` | `/dev/ttyUSB0` |
| `baud_rate` | 115200 |
| `wheel_radius` | 0.0375 m |
| `track_width` | 0.341 m |
| `ticks_per_rev` | **293** (measured: ~2929 ticks over 10 revolutions) |
| `max_pwm` | **249** (98 % of 255 — DBH-1B hardware limit) |
| `max_velocity` | 0.5 m/s — **estimate, needs ground tuning** |
| `cmd_vel_timeout` | 0.5 s |

Kinematic conversion in `motor_bridge.py`:
- `v_left  = linear − angular · track_width / 2`
- `v_right = linear + angular · track_width / 2`
- `pwm = clamp(v · max_pwm / max_velocity, ±max_pwm)`

Per-tick distance: `2π · wheel_radius / ticks_per_rev = 2π · 0.0375 / 293 ≈ 8.04 × 10⁻⁴ m/tick`.

---

## 8. EKF tuning

> Source: `robot_imu/config/ekf.yaml` — `ekf_filter_node` (robot_localization)

| Parameter | Value |
|---|---|
| `frequency` | 50 Hz |
| `sensor_timeout` | 0.1 s |
| `two_d_mode` | `true` |
| `publish_tf` | `true` |
| `odom_frame` | `odom` |
| `base_link_frame` | `base_footprint` |
| `world_frame` | `odom` |

State channels enabled (`config` vector indices `[x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz, ax, ay, az]`):

| Source | Channels used |
|---|---|
| `odom0 = /odom` | `vx`, `vy`, `wz` |
| `imu0 = /imu` | `wz`, `ax`, `ay`  (with `imu0_remove_gravitational_acceleration: true`) |

Neither input is treated as differential.

---

## 9. Nav2 tuning

> Source: `robot_description/config/nav2_params.yaml`

**DWB local planner:**

| Parameter | Value |
|---|---|
| `controller_frequency` | 10 Hz |
| `max_vel_x` / `min_vel_x` | 0.30 / 0.0 m/s |
| `max_vel_theta` | 1.0 rad/s |
| `acc_lim_x` / `acc_lim_theta` | 2.5 m/s² / 3.2 rad/s² |
| `decel_lim_x` / `decel_lim_theta` | −2.5 / −3.2 |
| `vx_samples` × `vtheta_samples` | 20 × 20 (vy 5) |
| `sim_time` | 1.7 s |
| Critic scales | RotateToGoal 32, GoalAlign 24, PathAlign 32, PathDist 32, GoalDist 24, BaseObstacle 0.02 |

**Costmaps** (both):

| Parameter | Value |
|---|---|
| `resolution` | 0.05 m |
| `robot_radius` | 0.25 m |
| `inflation_layer.inflation_radius` | 0.55 m |
| `inflation_layer.cost_scaling_factor` | 3.0 |

**Local costmap:** rolling window 3 × 3 m, `update_frequency` 5 Hz, `publish_frequency` 2 Hz, frame `odom`. Voxel layer with `z_resolution` 0.05 m, `z_voxels` 16 (max obstacle height 2.0 m), source `/scan` (`obstacle_max_range` 2.5 m, `raytrace_max_range` 3.0 m).

**Global costmap:** frame `map`, `update_frequency` 1 Hz, `track_unknown_space: true`, plugins = static + obstacle + inflation.

**AMCL:**

| Parameter | Value |
|---|---|
| `min_particles` / `max_particles` | 500 / 2000 |
| `update_min_d` | 0.25 m |
| `update_min_a` | 0.2 rad |
| `transform_tolerance` | 1.0 s |
| `laser_model_type` | `likelihood_field` |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` |
| `alpha1`–`alpha5` | 0.2 each |
| `max_beams` | 60 |

**Planner:** `nav2_navfn_planner::NavfnPlanner`, `tolerance` 0.5 m, `use_astar: false` (Dijkstra), `allow_unknown: true`, `expected_planner_frequency` 20 Hz.

**Behavior server:** plugins = spin, backup, drive_on_heading, assisted_teleop, wait. `cycle_frequency` 10 Hz, max/min rotational velocity 1.0 / 0.4 rad/s, rotational accel 3.2 rad/s².

**Velocity smoother:** `smoothing_frequency` 20 Hz, `max_velocity` [0.3, 0.0, 1.0], `max_accel` [2.5, 0.0, 3.2], odom topic `/odometry/filtered`.

**Robot base frame everywhere:** `base_footprint` (matches the EKF and SLAM configs — using `base_link` causes the map to drift in RViz).

---

## 10. SLAM Toolbox tuning

> Source: `robot_description/config/slam_toolbox.yaml` — `async_slam_toolbox_node`

| Parameter | Value |
|---|---|
| `mode` | `mapping` |
| `solver_plugin` | `solver_plugins::CeresSolver` (sparse normal Cholesky, Schur-Jacobi, LM) |
| `odom_frame` / `map_frame` / `base_frame` | `odom` / `map` / `base_footprint` |
| `scan_topic` | `/scan` |
| `minimum_travel_distance` | 0.1 m |
| `minimum_travel_heading` | 0.175 rad |
| `scan_buffer_size` | 10 |
| `scan_buffer_maximum_scan_distance` | 5.0 m |
| `do_loop_closing` | `true` |
| `loop_search_maximum_distance` | 3.0 m |
| `loop_match_minimum_response_fine` | 0.45 |
| `correlation_search_space_dimension` / `_resolution` | 0.5 m / 0.01 m |
| `loop_search_space_dimension` / `_resolution` | 8.0 m / 0.05 m |

---

## 11. Calibration & tuning notes

- **Encoder calibration** — measured 2929 ticks over 10 wheel revolutions → 293 ticks/rev. Refine if odometry drifts after long drives.
- **`max_velocity`** — currently 0.5 m/s as an estimate. Drive a known distance under load and adjust.
- **Right-motor encoder direction** is flipped in the firmware ISR because the right gearbox is mounted mirror-imaged. Do **not** flip it again in `motor_bridge.py`.
- **AMCL transform_tolerance 1.0 s** is generous — tighten to ~0.3 s once odometry latency is well-characterised.
- **Costmap inflation_radius 0.55 m vs. robot_radius 0.25 m** gives a 0.30 m buffer; reduce if the robot refuses to fit through doorways.

---

## 12. GY-91 register quick reference

> Source: `robot_imu/imu_node.py`

| Register | Address | Value written | Effect |
|---|---|---|---|
| `PWR_MGMT_1` | 0x6B | `0x00` | clear sleep bit, internal clock |
| `ACCEL_CFG`  | 0x1C | `0x00` | range ±2 g (16384 LSB/g) |
| `GYRO_CFG`   | 0x1B | `0x00` | range ±250 °/s (131 LSB/°/s) |
| `ACCEL_XOUT` | 0x3B | (read) | start of 14-byte burst: AX, AY, AZ, T, GX, GY, GZ |
| `TEMP_OUT`   | 0x41 | (read) | T_C = raw / 333.87 + 21.0 |
| `GYRO_XOUT`  | 0x43 | (read) | within the same 14-byte burst |

Constants used in code: `GRAVITY = 9.80665 m/s²`, `DEG_TO_RAD = π/180`.
