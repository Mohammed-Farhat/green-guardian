# Green Guardian — Full Technical Specification

This document is the single, exhaustive technical reference for the **Green Guardian** autonomous garbage-collection robot. It describes every subsystem in implementation-level detail: hardware, firmware, ROS 2 graph, control algorithms, mapping/localisation math, perception pipeline, the gripper FSM, mission-state machine, and the web dashboard.

The robot has one job: drive around an indoor arena, detect a piece of waste with a camera, decide whether it is **organic** or **non-organic**, pick it up with a 6-DOF gripper arm, and drop it into the matching on-board bin.

---

## 1. System Overview

### 1.1 What the robot does, end-to-end

1. **Mapping phase** – Driven manually with a teleop keyboard while `slam_toolbox` builds a 2-D occupancy grid of the arena from RPLidar scans + wheel-encoder odometry + IMU yaw rate.
2. **Map save** – `nav2_map_server map_saver_cli` writes the map to `arena_map.yaml` / `arena_map.pgm`.
3. **Autonomous phase** – Nav2 (AMCL + NavFn global planner + DWB local planner) drives the robot to a sequence of waypoints / a random wander goal.
4. **Perception** – A laptop-side YOLOv11 node (running on the same ROS 2 domain via Wi-Fi) classifies every camera frame published by the Pi-side `v4l2_camera` driver.
5. **Mission FSM** – When a stable detection is confirmed, `mission_node` cancels Nav2 motion (zero `cmd_vel` for 2 s), then sends a 2-byte serial command (`G` + bin selector) to **Arduino #2**.
6. **Gripper FSM** – Arduino #2 runs a six-state FSM driving 6× MG996R servos: lower → close → lift → swing-to-bin → open → return-home. An ultrasonic sensor provides depth feedback during the lowering phase, and a 3-attempt retry loop is used if the close-jaws step does not register contact.
7. **Cooldown** – After the gripper completes the cycle, `mission_node` ignores further detections for 10 s so the robot can drive away from the just-picked area.
8. **Bin-full handling** – When either bin level (published on `/bins/organic` and `/bins/non_organic`) crosses 100 %, the mission node sends Nav2 a goal at a predefined dump point on the saved map and stops.

### 1.2 Compute split

| Computer            | Role                                                                                |
| ------------------- | ----------------------------------------------------------------------------------- |
| **Raspberry Pi 5**  | Drivers + low-latency control: motor bridge, IMU + EKF, RPLidar driver, SLAM/AMCL/Nav2, mission FSM, bin levels, system monitor, rosbridge server, v4l2 camera driver. |
| **Arduino Uno #1**  | Motor PWM + quadrature encoder reading, watchdog-protected.                          |
| **Arduino Uno #2**  | Gripper FSM (6 servos + HC-SR04 ultrasonic).                                         |
| **Laptop**          | YOLOv11 inference, RViz2 visualisation, Node.js/Express + MongoDB backend, React frontend, noVNC server that streams RViz into the dashboard. |

Pi and laptop are on the same LAN with `ROS_DOMAIN_ID=0`. The laptop subscribes to `/camera/image_raw` over DDS and publishes back on `/garbage_detection`. The dashboard backend connects over `rosbridge_websocket` on port 9090.

### 1.3 Hardware ground truth

* **Chassis**: two 40 × 30 cm plexiglass plates, 4 mm thick, separated by 58 mm standoffs. Ground clearance 53 mm, total height 66 mm.
* **Wheels**: 4× JGA-25-370 DC gear-motors, 75 mm diameter (radius 37.5 mm), 28.5 mm width. Track width 341 mm centre-to-centre, wheelbase 290 mm.
* **Motor driver**: DBH-1B dual-channel H-bridge. Two motors per side are wired in parallel and driven by a single PWM channel — the robot is **differential drive**, not 4-wheel-independent. Max PWM clamped at 249 (~98 % of 255) to keep the H-bridge inside its thermal envelope.
* **Encoders**: incremental quadrature on each rear wheel; only one wheel per side is read. Empirically calibrated to **293 ticks per revolution** (`2929 ticks / 10 rev`).
* **LiDAR**: Slamtec RPLidar A1, 12 m range, 360°, ~5.5 Hz. Mounted on a 145 mm standoff at chassis position (X = +99 mm, Y = +106 mm) so it clears the gripper.
* **Camera**: Logitech C920, 640 × 480 @ 15 fps (intentionally throttled — see §5.1), pitched 20° downward, mounted 150 mm above ground at the front centre.
* **IMU**: GY-91 module (MPU-6500 used; magnetometer ignored). I²C address 0x68 on bus 1.
* **Ultrasonic**: HC-SR04 mounted on the gripper end-effector for final-approach distance feedback.
* **Gripper**: custom 6-DOF metal arm, 6× MG996R servos, controlled directly by Arduino #2.
* **Bins**: two front-mounted boxes — left = organic (green), right = non-organic (yellow), dimensions 80 × 120 × 66 mm.
* **Power**: 5 V battery for the Pi/logic; 12 V LiPo for motors + servos. Battery voltage is measured via an ADS1115 16-bit ADC over I²C, with a 10 kΩ / 3.3 kΩ voltage divider (scale 4.03) to bring 12 V into the 0–3.3 V ADC range.

---

## 2. ROS 2 Graph

### 2.1 Nodes and their packages

```
ros2_ws/src/
├── robot_description     # URDF (XACRO) + RViz preview + nav2.launch.py + slam.launch.py
├── robot_motors          # motor_bridge (Pi)  + motor_control.ino (Arduino #1)
├── robot_imu             # imu_node + ekf_node config (robot_localization)
├── robot_slam            # SLAM bringup composition launch
├── robot_navigation      # Nav2 bringup composition launch + random_wanderer.py
├── robot_vision          # yolo_node (laptop) + temporal_filter + mission_node (Pi)
├── robot_dashboard       # system_monitor_node + bin_levels_node + odom_node (placeholder)
└── rplidar_ros           # vendored Slamtec driver
```

### 2.2 Topic / TF map

| Topic                          | Type                            | Publisher                | Subscribers                       |
| ------------------------------ | ------------------------------- | ------------------------ | --------------------------------- |
| `/scan`                        | `sensor_msgs/LaserScan`         | `rplidar_node`           | `slam_toolbox`, `amcl`, costmaps  |
| `/odom`                        | `nav_msgs/Odometry`             | `motor_bridge` (Pi)      | `ekf_filter_node`, Nav2, dashboard|
| `/imu`                         | `sensor_msgs/Imu`               | `imu_node`               | `ekf_filter_node`                 |
| `/joint_states`                | `sensor_msgs/JointState`        | `motor_bridge`           | `robot_state_publisher`           |
| `/cmd_vel`                     | `geometry_msgs/Twist`           | DWB / teleop / mission   | `motor_bridge`                    |
| `/camera/image_raw`            | `sensor_msgs/Image`             | `v4l2_camera_node`       | `yolo_node`                       |
| `/camera/yolo_annotated`       | `sensor_msgs/Image`             | `yolo_node`              | RViz / debug                      |
| `/garbage_detection`           | `std_msgs/String`               | `yolo_node`              | `mission_node`                    |
| `/system/cpu_temp` etc.        | `std_msgs/Float32`              | `system_monitor_node`    | dashboard                         |
| `/bins/{organic,non_organic}`  | `std_msgs/Float32`              | `bin_levels_node`        | dashboard, mission_node           |

**TF tree** (steady state, AMCL phase):
```
map ──(amcl)──> odom ──(ekf_filter_node)──> base_footprint
                                                ├── base_link        (fixed)
                                                ├── imu_link
                                                ├── lidar_standoff_link
                                                │       └── laser_link
                                                ├── camera_link
                                                │       └── camera_optical_link
                                                ├── bin_left_link
                                                ├── bin_right_link
                                                └── 4× wheel_*_link   (continuous joints, driven by /joint_states)
```

Two important details:
* `motor_bridge` exposes a `publish_tf` parameter that is set to **False** in the production launch — the EKF owns `odom → base_footprint` so the two TF publishers do not fight.
* `laser_joint` has a `rpy="0 0 3.14159"` rotation: the RPLidar is physically mounted facing backwards, so the URDF rotates it 180° around Z to put scan 0° in front of the robot.

### 2.3 QoS choices that matter

* `/odom` is **RELIABLE** because the Nav2 controller_server and `ekf_filter_node` both subscribe with the default reliable profile. Publishing it `BEST_EFFORT` caused Nav2 to silently drop every message and log `incompatible QoS / RELIABILITY_QOS_POLICY` — this is why the publisher in `motor_bridge.py` explicitly declares a reliable profile.
* `/cmd_vel` is **BEST_EFFORT** — both teleop and Nav2 accept it.
* `/camera/image_raw` and `/camera/yolo_annotated` are **BEST_EFFORT, KEEP_LAST(5)** — video data is high-rate and stale frames are useless.
* `/map` is **RELIABLE + TRANSIENT_LOCAL** so that late-joining subscribers (random_wanderer, costmaps) receive the latched map.

---

## 3. Hardware Description (URDF / XACRO)

The robot model lives in `robot_description/urdf/` and is split into four files for maintainability:

* `dimensions.xacro` — every measurement as a `<xacro:property>`. **All other XACROs reference these properties — no number is hard-coded twice.** The top of the file documents derivations such as `base_link_z = clearance + chassis_height/2 = 0.086 m`.
* `materials.xacro` — visual RGBA colours (translucent grey chassis, brass standoff, blue PCB, green/yellow bins).
* `base.xacro` — `base_footprint`, `base_link`, four wheels, IMU.
* `sensors.xacro` — LiDAR standoff + body, camera + optical frame, two bins.

### 3.1 Frame choices

* **`base_footprint`** is a virtual ground-plane link with no geometry — Nav2 / AMCL require it. The fixed joint to `base_link` lifts the chassis box centre to Z = 0.086 m.
* **`base_link`** is one `<box>` representing the entire two-plate stack. Modelling each plate + standoff separately was deemed unnecessary because nothing about the autonomy stack cares about the inter-plate volume.
* **`camera_optical_link`** applies the ROS-camera convention (Z forward, X right, Y down) via `rpy="-pi/2 0 -pi/2"`. All `sensor_msgs/Image` headers use this frame so YOLO bounding boxes are interpretable in the optical convention.

### 3.2 Wheel macro

The four wheels are generated by an XACRO macro called four times with `(x, y)` parameters. Each wheel cylinder is rotated `pi/2` around X so its axis is along Y (left-right), and each joint is `continuous` with `axis="0 1 0"` so the wheel can spin freely. The macro centralises inertia (m = 0.2 kg, solid-cylinder formula).

### 3.3 What is intentionally **not** in the URDF

* **Gripper arm** — excluded so the URDF stays simple for RViz and so Nav2's collision checking treats the chassis as a single box. The gripper is driven entirely outside ROS, by Arduino #2.
* **Ultrasonic sensor** — not load-bearing for navigation; only used inside the gripper FSM.

---

## 4. Locomotion: Motor Bridge + Arduino #1

### 4.1 Differential-drive kinematics

`/cmd_vel` carries `linear.x` (m/s, forward) and `angular.z` (rad/s, yaw, CCW positive). The bridge converts these to per-wheel linear velocities with the classic differential equations:

```
v_left  = linear.x − angular.z · (track_width / 2)
v_right = linear.x + angular.z · (track_width / 2)
```

with `track_width = 0.341 m`. Each `v_*` is then mapped to a PWM duty cycle by a linear scaling:

```
scale     = max_pwm / max_velocity                  # 249 / 0.543 ≈ 459 PWM·s/m
pwm_left  = round(v_left  · scale)
pwm_right = round(v_right · scale)
```

### 4.2 Stiction compensation

Below roughly PWM 60–70 the JGA gear-motors stall on the ground (gearbox friction + cable drag). DWB issues very small rotational `cmd_vel` values during fine alignment, which would mathematically map to PWM 12–30 and the robot would silently not move. To fix this, every non-zero PWM is bumped up to `min_pwm = 70`:

```python
def _apply_stiction(pwm):
    if pwm == 0:           return 0
    if abs(pwm) < min_pwm: return  min_pwm if pwm > 0 else -min_pwm
    return pwm
```

Both stiction-corrected outputs are then hard-clamped to `[-249, +249]` before being shipped over serial.

### 4.3 Serial protocol (Pi ↔ Arduino #1)

ASCII, line-terminated, 115 200 baud:

| Direction | Frame                       | Meaning                                  |
| --------- | --------------------------- | ---------------------------------------- |
| Pi → Uno  | `M:LEFT,RIGHT\n`            | New motor command (PWM signed integers). |
| Uno → Pi  | `O:LEFT,RIGHT\n`            | Encoder tick delta (20 Hz).              |
| Uno → Pi  | `OK\n`                      | Command acknowledged.                    |
| Uno → Pi  | `E:<msg>\n`                 | Parser error (e.g. malformed M frame).   |
| Uno → Pi  | `GG:ready\n`                | Boot banner.                             |

A fixed 32-byte buffer is used on the Arduino side to avoid heap fragmentation. The Pi side decodes lines in a background thread (`_serial_read_loop`) so blocking reads never starve the ROS executor.

### 4.4 Watchdogs

There are two independent watchdogs — one on each side of the serial link:

* **Pi side** (`watchdog_callback` at 10 Hz): if no `/cmd_vel` has been received within `cmd_vel_timeout = 0.5 s`, the bridge sends `M:0,0\n` once and remembers the motors are stopped.
* **Arduino side** (`WATCHDOG_MS = 500` in `motor_control.ino`): if no `M:` frame has been received within 500 ms, the Arduino zeroes all four PWM channels itself.

Either watchdog alone would be sufficient. They are deliberately redundant: if the USB cable is unplugged mid-run, the Arduino watchdog still fires; if the Arduino hangs but USB is up, the Pi watchdog still fires (and a serial reconnect loop retries every 2 s).

### 4.5 Wheel-odometry math

Arduino #1 publishes an integer **tick delta** every 50 ms (= 20 Hz). The Pi converts these to metres travelled per wheel:

```
m_per_tick = (2 · π · wheel_radius) / ticks_per_rev
           = (2 · π · 0.0375) / 293
           ≈ 0.000804 m/tick
```

Then differential-drive forward kinematics with **mid-point heading integration**:

```
d_left   = left_ticks  · m_per_tick
d_right  = right_ticks · m_per_tick
d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / track_width

mid_theta = theta + d_theta / 2          # half-step yaw for midpoint integration
theta    += d_theta
x        += d_center · cos(mid_theta)
y        += d_center · sin(mid_theta)
theta     = atan2(sin(theta), cos(theta))   # wrap to (-π, π]
```

Mid-point integration (vs. forward-Euler with the start-of-step heading) approximately halves the heading error on a curved path, which matters because the error is integrated forever.

The published `/odom` message includes:
* Pose `(x, y, qz=sin(θ/2), qw=cos(θ/2))` — Z-only quaternion since the robot is planar.
* Twist `(linear.x = d_center/dt, angular.z = d_theta/dt)` where `dt` is the **actual** wall-clock delta between encoder messages, not a fixed 50 ms (which would amplify error if the Arduino USB loop misses a tick).
* Diagonal covariance: `σ²_x = σ²_y = 0.01`, `σ²_yaw = 0.05`. These are tuned to be slightly looser than the EKF's IMU yaw-rate covariance so the EKF prefers the IMU for heading and the encoders for translation.

`JointState` is also published with cumulative wheel angles `(left_angle, right_angle)` for `robot_state_publisher`, so RViz shows the wheels spinning in real time.

### 4.6 Arduino #1 firmware (`motor_control.ino`)

* **Two interrupts** on pins 2 (right A) and 3 (left A), RISING edge. Direction is decided by reading the B channel: `HIGH → ++`, `LOW → --`. The right encoder ISR has its sign **flipped in software** because the right motor is physically mirrored on the chassis — this is cheaper than rewiring.
* **`setLeft(speed) / setRight(speed)`** drive forward and reverse PWM channels via `analogWrite`. The DBH-1B's ENA/ENB pins are tied HIGH at boot, so signed speed is encoded purely in which of `IN1*` / `IN2*` carries the PWM and which carries 0.
* **Atomic encoder read**: before assembling each `O:` frame the firmware disables interrupts, copies `enc_left` / `enc_right` to local vars, resets them, then re-enables interrupts. This avoids torn reads when an ISR fires mid-print.

### 4.7 Parameters file (`hardware_params.yaml`)

The yaml lives with the package and is loaded by the launch file. Anything the operator might want to tune on hardware is exposed: serial port, baud, wheel radius / track width, ticks/rev, max PWM, **min PWM (stiction)**, max velocity, watchdog timeout.

---

## 5. Perception: YOLOv11 + Temporal Filter

### 5.1 Camera driver

`v4l2_camera_node` is the standard ROS driver. The launch file forces:
* `image_size: [640, 480]` — small enough to keep YOLO inference cheap.
* `time_per_frame: [1, 15]` — **15 fps cap**. Above this the laptop YOLO node falls behind and queue depth grows.
* `pixel_format: YUYV` — what the C920 reports natively; `cv_bridge` converts to BGR on the laptop.
* `camera_frame_id: camera_optical_link` — makes `/camera/image_raw` headers consistent with the URDF.

### 5.2 Model

* **YOLOv11 Nano** (`best.pt`, ~5 MB), trained on a custom dataset of ~400 university-campus photos annotated in Label Studio (YOLO bounding-box format).
* **Two classes**: `organic` (class_id 0), `non_organic` (class_id 1).
* **Best mAP**: ~90 % on a held-out validation split. We also trained YOLOv11 Medium; Nano was chosen because Medium does not hit real-time inference rates on the Pi CPU and the laptop runs it more comfortably while saving battery.

### 5.3 Inference scheduling

YOLO runs in the same Python process as the ROS node but inference itself is a single Ultralytics `self.model(frame, ...)` call inside the `/camera/image_raw` callback. The QoS is `BEST_EFFORT, KEEP_LAST(5)`: if the laptop falls behind, frames are dropped at the queue rather than backing up. Empirically on the laptop CPU we sustain ~12 fps inference at 640×480 with the Nano model.

### 5.4 Why a temporal filter

YOLO sometimes briefly latches onto a piece of background (a tile pattern, a wall outlet) with one frame of high confidence. A single false positive would cause the robot to stop and attempt a pickup of nothing. To prevent this, we wrap the model in a **temporal consistency filter** (`robot_vision/temporal_filter.py`) and only publish detections that survive multiple frames.

### 5.5 Algorithm (TemporalConsistencyFilter)

It is a small IoU-based tracker. Parameters:

| Param          | Default in `yolo_node` | Meaning                                                             |
| -------------- | ---------------------- | ------------------------------------------------------------------- |
| `required_hits`| 5                      | Frames a detection must survive before being **confirmed**.         |
| `iou_threshold`| 0.4                    | IoU above which two frames are considered the same physical object. |
| `max_misses`   | 2                      | Frames a tracked object may go missing before being dropped.        |

Per-frame update logic (`update()`):

1. **Match**: for each new raw detection, find the highest-IoU existing track. If `IoU ≥ 0.4`, claim it, increment its `hits`, reset `misses`. If `hits ≥ required_hits`, flag it `confirmed`.
2. **Spawn**: any raw detection that didn't match starts a brand-new track with `hits=1`. It is **not** published until it survives `required_hits-1` more frames.
3. **Age**: every existing track that was not matched this frame has its `misses` incremented. Tracks with `misses > max_misses` are dropped.
4. **Return**: only tracks where `confirmed == True` are returned. These are what `yolo_node` publishes.

IoU itself is computed by converting normalised `[xc, yc, w, h]` boxes to corners, taking the intersection rectangle's area, and dividing by the union (with a `1e-6` epsilon to avoid divide-by-zero on zero-area boxes):

```
iou = inter_area / (area_a + area_b - inter_area + 1e-6)
```

### 5.6 Small-object whitelist bypass

A second mechanism handles the case where a tiny real object (e.g. a cigarette butt at distance) is detected for one frame and then disappears as we drive past. Boxes whose pixel area is **less than 1 % of the frame area** skip the temporal filter entirely and are accepted on the first frame. They are drawn in **purple** with an `[S]` (small) prefix in the annotated image; normal confirmed detections are drawn in **green**.

```python
if pixel_area < 0.01 * frame_area:
    whitelisted.append(det)   # bypass filter
else:
    normal_detections.append(det)
confirmed = self._filter.update(normal_detections)
all_accepted = confirmed + whitelisted
```

The winning `class_name` (highest confidence among `all_accepted`) is the published `/garbage_detection` payload. If nothing is accepted this frame, `"none"` is published — which the mission node ignores.

### 5.7 Annotated debug stream

For every frame, `yolo_node` republishes the same image with boxes + class labels overlaid on `/camera/yolo_annotated`. This is what is shown in the RViz `Image` panel and is also what gets piped through noVNC to the dashboard Tracking tab.

---

## 6. Mission Control + Gripper FSM

### 6.1 `mission_node` (Pi)

`mission_node` is the brain that converts "detection confirmed" into "robot stops + gripper picks". It is intentionally minimal — a thin coordinator between Nav2 output and the Arduino-#2 FSM.

State:

```python
self._in_cooldown = False
self._state_lock  = threading.Lock()
```

#### 6.1.1 Callback flow

```
/garbage_detection ──►  _detection_callback(msg)
                          │  label = msg.data
                          │  if label in {"organic","non_organic"} and not in cooldown:
                          │      _in_cooldown = True
                          │      spawn daemon thread → _stop_and_trigger(label)
```

Splitting the heavy work into a daemon thread is critical — the gripper sequence blocks for ~10 s, and doing it inside the ROS callback would freeze the executor.

#### 6.1.2 `_stop_and_trigger` (background thread)

Three phases:

1. **Stop phase (2 s)**. Publish a zero `Twist` to `/cmd_vel` every 100 ms for 2 s. The motor_bridge's stiction logic returns 0 for input 0, so the wheels stop hard. Nav2's controller server keeps trying to send its own `/cmd_vel`, but our zero messages are produced at 10 Hz and are interleaved — the watchdog effectively wins. (When this becomes a problem we will additionally call Nav2's `cancel_goal` action — currently the cooldown is long enough that it doesn't matter.)
2. **Trigger phase**. Open the gripper serial port (`/dev/ttyUSB1`, 9600 baud) and send two bytes:
   * `G` — the universal "start the pick sequence" trigger.
   * `O` or `N` — bin selector. `O` (organic) routes the arm to the left bin; `N` (non-organic) routes to the right bin.
3. **Cooldown (10 s default)**. `time.sleep(self.cooldown)` then `_in_cooldown = False`. Any detections that arrive during cooldown are ignored, which lets Nav2 resume its goal and physically drive past the now-collected object so we don't re-trigger on the empty patch of floor.

The serial port is opened **inside a `with serial.Serial(...)`** context manager so the port is released even if the gripper Arduino is unplugged mid-run.

### 6.2 Arduino #2 — Gripper FSM

The gripper Arduino runs a six-state finite-state machine. State transitions are time-driven plus one external trigger byte from the Pi.

```
        ┌──────────┐                ┌───────────┐
  G,O ─►│   IDLE   │──── G,O/N ────►│  LOWER    │  arm pitches forward + down
        └──────────┘                └─────┬─────┘
              ▲                            │ ultrasonic < target_dist  (≤ ~5 cm)
              │                            ▼
              │                       ┌──────────┐
              │                       │  CLOSE   │  jaw servo to grip angle
              │                       └─────┬────┘
              │                            │ 600 ms settle  (3-attempt retry on slip)
              │                            ▼
              │                       ┌──────────┐
              │                       │  LIFT    │  reverse pitch back to "up" pose
              │                       └─────┬────┘
              │                            │
              │                            ▼
              │                  ┌────────────────────┐
              │                  │  SWING_TO_BIN(O|N) │  rotate base servo to side
              │                  └─────────┬──────────┘
              │                            │
              │                            ▼
              │                       ┌──────────┐
              │                       │  RELEASE │  open jaws above target bin
              │                       └─────┬────┘
              │                            │
              │                            ▼
              │                       ┌──────────┐
              └───── done ────────────│  HOMING  │  return base to forward, lift to neutral
                                      └──────────┘
```

Servo allocation (6× MG996R):

| Joint              | Role                          |
| ------------------ | ----------------------------- |
| Base (yaw)         | Swing arm left / centre / right (bin selection). |
| Shoulder (pitch)   | Lower toward ground / raise to neutral.          |
| Elbow              | Fine pitch — lengthens reach when lowering.       |
| Wrist roll         | Keep gripper level as shoulder/elbow change.      |
| Wrist pitch        | Final approach tilt onto the object.              |
| Jaw                | Open ↔ close on the object.                       |

#### 6.2.1 LOWER → CLOSE: ultrasonic feedback

In LOWER, the FSM ramps the shoulder + elbow servos forward in small steps (10 ms per °) while reading the HC-SR04. When the measured distance to the object drops below the target threshold (~5 cm — tuned per object size), the FSM stops the descent and transitions to CLOSE.

#### 6.2.2 Three-attempt retry

When the jaw servo finishes closing, the firmware measures the **stall current pattern** (read indirectly: the jaw servo's position vs commanded position, since MG996R has internal position feedback that we expose by leaving the servo in attached mode and checking whether the command was reached within tolerance). If the jaws closed **all the way** without resistance, the FSM assumes the grip slipped: it opens the jaw, lifts ~2 cm, and re-attempts CLOSE. After **3 failed attempts** the FSM:

* Aborts the cycle (skips SWING / RELEASE).
* Returns to HOMING immediately.
* On the Pi side, when `mission_node` next runs Nav2 it will mark the object's coordinates as a **temporary obstacle** in a custom layer and avoid that pose for the rest of the run, so the robot doesn't sit forever next to an un-pickable object.

#### 6.2.3 Serial command parser

Arduino #2 only listens for one command, `G`, followed by exactly one bin byte. The parser is intentionally simple:

```c
if (Serial.available() >= 2) {
  char trig = Serial.read();
  char bin  = Serial.read();
  if (trig == 'G' && (bin == 'O' || bin == 'N'))
      runPickSequence(bin);   // blocks until done
  // any other combination → ignored, parser resyncs on next byte
}
```

No acknowledgement is sent back — `mission_node` only needs the gripper to have *started*; the cooldown handles synchronisation.

### 6.3 What happens **during** the pick

Looking at the robot from outside:

1. Robot is driving on a Nav2 path. Camera streams to laptop → YOLO confirms `non_organic` 5 frames in a row.
2. `mission_node` sends 20 zero `cmd_vel` messages in 2 s. Robot brakes to a halt.
3. Mission node writes `G` `N` to Arduino #2 over `/dev/ttyUSB1`. Arduino #2's FSM starts.
4. Arm lowers, ultrasonic confirms ~5 cm to object, jaws close, lift, swing right, drop in non-organic bin, home.
5. Mission node has been sleeping in cooldown the whole time. After 10 s it releases the lock.
6. Nav2's controller server has been retrying its goal and the moment our zero-`cmd_vel` spam stops, the path resumes — the robot drives past the (now empty) detection patch, cooldown prevents re-triggering.

This is the behaviour described in the FYP report as: *"when the YOLO detects, the robot stops and the gripper works."* Implementation note: the integration between YOLO confirmation and `mission_node`'s ultrasonic-blocked descent is **the part still being polished** — everything before and after that point already works end-to-end.

---

## 7. Mapping: SLAM Toolbox

### 7.1 What SLAM Toolbox is doing

`slam_toolbox` (async mode) runs **scan-matching SLAM** with **pose-graph optimisation** and **loop closure**. Internally:

1. Each new `/scan` is matched to the previous one with a fine scan-matcher (correlation-based, parameterised by `correlation_search_space_*`). The result is a relative `(Δx, Δy, Δθ)` between scans.
2. A pose graph is built, with each scan being a vertex and each relative transform an edge.
3. Whenever the robot re-enters a previously mapped area (detected by `loop_search_*`), a loop-closure constraint is added.
4. The whole graph is optimised by a **Ceres solver** with the `SPARSE_NORMAL_CHOLESKY` linear solver and `LEVENBERG_MARQUARDT` trust strategy. This refines all past poses jointly to be consistent with both odometry and loop closures.
5. The resulting occupancy grid is published on `/map`.

### 7.2 Key configuration choices (`slam_toolbox.yaml`)

* **`mode: mapping`** — running in mapping (not localisation) mode; the AMCL phase is a separate launch.
* **`max_laser_range: 12.0`** — matches the RPLidar A1 datasheet limit. Above this the lidar returns garbage.
* **`min_laser_range: 0.15`** — RPLidar A1 has a 15 cm dead-zone. Including closer returns produces noise points at the chassis.
* **`minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.0524`** (3°) — how far the robot must move before a new keyframe is added. The heading threshold was originally 10° (the slam_toolbox default), which caused ghost walls during in-place rotations (the gap between keyframes was so large that the scan-matcher couldn't reliably stitch them, and the costmap got "doubled walls"). Dropping to 3° fixed it.
* **`transform_timeout: 0.2`** — small (default is 1 s). Higher values let slam_toolbox use stale TFs during turns, which causes ghost walls.
* **`loop_search_maximum_distance: 3.0`** + **`loop_match_minimum_chain_size: 10`** — look for loop closures within 3 m, but only commit one if at least 10 consecutive keyframes match. This avoids spurious loops in small arenas.
* **`scan_buffer_size: 10`**, **`use_scan_barycenter: true`** — average 10 scans worth of barycentres for the running pose estimate, smoothing single-scan noise.

### 7.3 SLAM bringup composition

`robot_slam/launch/slam_bringup.launch.py` includes (in order):

1. `robot_motors/motor_bridge.launch.py` — wheels + encoder odometry.
2. `robot_imu/imu.launch.py` — IMU + EKF (EKF publishes `odom → base_footprint`).
3. `rplidar_ros/rplidar_a1_launch.py` with `frame_id:=laser_link` to match the URDF.
4. `robot_description/slam.launch.py` — `robot_state_publisher` + `slam_toolbox` + `nav2_lifecycle_manager` (used to bring `slam_toolbox` up cleanly — `bond_timeout: 0.0` disables liveness checking, which is what slam_toolbox expects).

Map saving is a manual step done in a separate terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f $HOME/arena_map
# → arena_map.yaml + arena_map.pgm
```

(`$HOME` is used, not `~/`, because `ros2 launch` does not expand `~` in `:=` arguments.)

---

## 8. Localisation: AMCL

### 8.1 Why AMCL not slam_toolbox

`slam_toolbox` is excellent for mapping but expensive for steady-state localisation. Once the map is built, we switch to `nav2_amcl`, which is a **particle filter (Monte Carlo Localisation)** specialised to known-map localisation.

### 8.2 Algorithm

* **Particles**: between 500 (min) and 5000 (max) `(x, y, θ)` hypotheses.
* **Motion model**: `DifferentialMotionModel`. After each `/odom` update, every particle is moved by the odometry delta plus Gaussian noise parameterised by `alpha1..alpha5` (0.2 each = moderate odometry noise — encoders are decent but slip on the smooth floor of the demo arena).
* **Sensor model**: `likelihood_field` (faster than the beam model, robust to occlusions, ignores `z_short`/`z_max`/`lambda_short` even though those are set). Each particle's weight is computed by ray-casting the scan against the map and applying:
  ```
  w = z_hit * N(d ; 0, σ_hit²) + z_rand * (1/z_max_range)
  ```
  with `z_hit = z_rand = 0.5`, `σ_hit = 0.2 m`, summed over `max_beams = 60` (the scan is subsampled to 60 rays for speed).
* **Resampling** happens every step (`resample_interval: 1`) using KLD-sampling to dynamically adjust the particle count between the min/max bounds, parameterised by `pf_err = 0.05` and `pf_z = 0.99`.
* **Update gating**: the filter only updates if the robot has moved **0.10 m or 0.1 rad** since the last update (`update_min_d`, `update_min_a`). The default was 0.25 m / 0.2 rad which gave coarse pose estimates and oscillating AMCL clouds — reducing them eliminated the visible "jumps" of the robot model in RViz.

### 8.3 Initial pose

`set_initial_pose: true` makes AMCL publish a `map → odom` TF at `(0,0,0)` immediately on startup. Without this, the global costmap can't activate (it waits forever for `map → base_footprint`). The operator then clicks **2-D Pose Estimate** in RViz to reset the particles to the robot's actual location; AMCL re-converges from laser data in ~2 s.

### 8.4 Why AMCL publishes TF, not pose

AMCL publishes the `map → odom` transform (`tf_broadcast: true`). It does **not** publish `map → base_footprint` directly; that comes via `map → odom → base_footprint`, with the EKF providing the latter. This is the canonical Nav2 setup and means that even if AMCL flickers, the robot's odometry-based pose is always available locally.

---

## 9. Sensor Fusion: IMU + EKF

### 9.1 IMU driver (`robot_imu/robot_imu/imu_node.py`)

We use the GY-91 module's **MPU-6500** (an MPU-6050-compatible 6-axis IMU). The magnetometer (AK8963) is ignored: outdoor mag fusion adds complexity that the indoor mission doesn't need.

* I²C bus 1, address `0x68`, 50 Hz sample rate.
* Accelerometer config `0x00` → ±2 g, scale 16384 LSB/g, gravity 9.80665 m/s².
* Gyro config `0x00` → ±250 °/s, scale 131 LSB/°/s.
* Burst-reads 14 bytes from `REG_ACCEL_XOUT` per cycle so accel + temp + gyro are atomically sampled.

#### 9.1.1 Gyro bias calibration on startup

Uncalibrated MPU-6500 gyros have a bias of 0.01–0.05 rad/s. Integrated by the EKF this looks like the robot slowly rotating even when stationary. To remove it, `imu_node` averages `calibration_samples = 200` (= 4 s at 50 Hz) of gyro readings during start-up, before any IMU message is published. The averaged values are subtracted from every subsequent reading. The constraint: **the robot must be still during boot** — operator instruction is "power on, don't touch for 5 s, then drive."

#### 9.1.2 Orientation field

`imu_node` does **not** integrate orientation. The `orientation_covariance[0]` is set to `-1.0`, which is the ROS convention for "ignore the orientation field". Downstream consumers (the EKF) read the **gyro yaw rate** instead.

### 9.2 robot_localization EKF (`ekf.yaml`)

The EKF state is the 15-D `[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]` standard for `robot_localization`. We configure it as **2-D** (`two_d_mode: true`), which clamps Z, roll, pitch and their derivatives to 0.

Sensor configurations are 15-element bool arrays. For our fusion:

```
odom0_config (use what from /odom):
  vx YES, vy YES, vyaw YES        — encoder twist
  all positions NO                 — let the filter integrate them itself

imu0_config (use what from /imu):
  vyaw YES                          — gyro yaw rate
  everything else NO
imu0_remove_gravitational_acceleration: true   (irrelevant since accels are off, but safe)
```

So the EKF fuses:
* **Encoder linear + angular velocity** with covariance `(0.01, 0.05)`.
* **Gyro yaw rate** with covariance `(0.001)`.

Because the gyro covariance is an order of magnitude smaller than the encoder yaw covariance, the EKF **trusts the gyro for heading rate** and **trusts the encoders for translation**, which is exactly the desired split: the encoders are great at distance but drift in heading on the smooth floor; the gyro is great at rotational rate but its integral drifts. Fused, the EKF gets the best of both.

The EKF runs at 50 Hz, publishes `odom → base_footprint`, and **owns** that TF (motor_bridge runs with `publish_tf: False` to avoid fighting it).

---

## 10. Autonomous Navigation: Nav2

### 10.1 Nodes started by `robot_description/launch/nav2.launch.py`

```
robot_state_publisher
map_server          (loads the .yaml passed via map:= argument)
amcl                (likelihood-field MCL — see §8)
controller_server   (DWB local planner → /cmd_vel)
planner_server      (NavFn global planner)
behavior_server     (spin / backup / drive_on_heading / wait recoveries)
bt_navigator        (executes the behaviour tree)
waypoint_follower   (FollowWaypoints action)
lifecycle_manager_navigation  (brings the above up in order)
```

All nine are managed by **one** lifecycle manager — `autostart: true` walks them through `unconfigured → inactive → active`. Order matters: `map_server` must be active before `amcl` can configure.

### 10.2 Global planner — NavFn (Dijkstra)

`planner_server` runs `nav2_navfn_planner::NavfnPlanner` with `use_astar: false` (we use Dijkstra, not A*). Dijkstra over a 5 cm-resolution grid is fast enough on the Pi and produces deterministic, shortest-distance paths — which look better in the demo than A*'s heuristic-biased paths in this small arena. `tolerance: 0.5 m` means the planner is happy if it can plan to within 50 cm of the goal (necessary because Nav2 sometimes receives goals just inside obstacle inflation).

### 10.3 Local planner — DWB (Dynamic Window Approach + Behaviour critics)

`controller_server` runs `dwb_core::DWBLocalPlanner`. DWB samples a 3-D velocity window (vx, vy, vθ), rolls each sample forward for `sim_time = 1.5 s`, and scores each trajectory with a weighted sum of critics. The critics we use, with scales:

| Critic         | Scale | Purpose                                                      |
| -------------- | ----- | ------------------------------------------------------------ |
| `RotateToGoal` | 32.0  | Reward turning to the goal yaw at the end.                   |
| `Oscillation`  | (default) | Penalise back-and-forth direction reversals.             |
| `BaseObstacle` | 0.02  | Penalise occupied cells. Low — we rely on inflation instead. |
| `GoalAlign`    | 24.0  | Reward heading toward the goal direction (lookahead 0.1 m).  |
| `PathAlign`    | 32.0  | Reward keeping aligned with the planned global path.         |
| `PathDist`     | 32.0  | Penalise distance from the planned path.                     |
| `GoalDist`     | 24.0  | Reward distance reduction to the goal.                       |

Velocity window:

```
max_vel_x: 0.15 m/s   (capped low for demo; max_velocity in motor_bridge is 0.543)
max_vel_theta: 1.5 rad/s
acc_lim_x: 1.0 m/s²
acc_lim_theta: 3.2 rad/s²
vx_samples: 20, vy_samples: 5, vtheta_samples: 20
```

`max_vel_theta` of 1.5 is high — needed because the chassis has heavy stiction and DWB needs rotational headroom to actually turn. Combined with `min_pwm = 70` in `motor_bridge` this gives reliable in-place rotation. The high `PathAlign` / `PathDist` scales make the robot hug the global path tightly, which looks deliberate in the demo.

### 10.4 Costmaps

Two costmaps, identical plugins minus the `static_layer`:

#### Local costmap
* `rolling_window: true`, `width × height = 3 × 3 m`, resolution 5 cm.
* Plugins: **`obstacle_layer`** + **`inflation_layer`**.
* Sensor source: `/scan` (LaserScan), `obstacle_min_range: 0.15` (skip the RPLidar A1 dead-zone), `obstacle_max_range: 2.5`, `raytrace_max_range: 3.0`.
* `obstacle_layer` plugin: was originally `VoxelLayer` (3-D), which silently consumed a `LaserScan` but produced a degenerate costmap. Switching to `ObstacleLayer` (the correct 2-D plugin) was one of the four critical Nav2 fixes for Jazzy.

#### Global costmap
* Full-map size, `static_layer` (the saved arena map) + `obstacle_layer` + `inflation_layer`.
* `track_unknown_space: true` — unknown cells are valid for the planner if `allow_unknown: true` is set on NavFn (it is).
* Same obstacle source as the local costmap.

#### Inflation layer
Both costmaps inflate obstacles using a decaying-cost field:
* `cost_scaling_factor: 5.0` — cost decays quickly.
* `inflation_radius: 0.20 m` — gradient is ~3 cm wide past the chassis's 0.17 m inscribed circle. The default 0.30 m painted most of the small arena blue (high cost) which restricted the planner.

#### Footprint
Both costmaps use a polygon footprint (`[[0.22, 0.17], [0.22, -0.17], [-0.22, -0.17], [-0.22, 0.17]]`) rather than `robot_radius: 0.25`. Using the polygon recovers ~10 cm of usable space along each wall, because the lethal/inscribed zone is the chassis half-width (0.17 m) rather than the diagonal (0.25 m).

### 10.5 Behaviour tree + recoveries

`bt_navigator` runs Nav2's default `navigate_to_pose` behaviour tree. This tree, on goal failure, falls through to the `behavior_server`'s plugins:
* `spin` — rotate 180° in place to re-perceive the environment.
* `backup` — drive ~0.3 m backward.
* `drive_on_heading` — drive forward on the current heading.
* `wait` — pause and let the costmap update.
* `assisted_teleop` — accept user input.

One Jazzy-specific gotcha was fixed: `navigators: ["navigate_to_pose"]` is loaded **once** (not twice). On Jazzy, each navigator independently loads `plugin_lib_names` into a *shared* `BehaviorTreeFactory`. With both `navigate_to_pose` and `navigate_through_poses` enabled, every BT plugin loads twice → `"ID [ComputePathToPose] already registered"` and the server crashes. `waypoint_follower` internally uses `navigate_to_pose` so it still works.

### 10.6 Velocity smoother

`velocity_smoother` is configured but not currently wired into the pipeline. It would sit between DWB's `/cmd_vel` output and `motor_bridge`, applying `max_accel` / `max_decel` per axis. We left it off because `motor_bridge`'s stiction layer already prevents the lurch DWB sometimes commands.

### 10.7 Combined autonomous bringup

`robot_navigation/launch/nav_bringup.launch.py` is the one-liner that brings up everything:

```bash
ros2 launch robot_navigation nav_bringup.launch.py map:=$HOME/arena_map.yaml
```

Internally it includes (in this order):

1. `motor_bridge.launch.py`
2. `imu.launch.py` (which itself spawns `imu_node` + `ekf_filter_node`)
3. `rplidar_a1_launch.py` (with `frame_id:=laser_link` to match the URDF)
4. `robot_description/launch/nav2.launch.py` (the 9 Nav2 nodes above + RSP)

`map` is a required argument; the launch file declares it with no default so missing it produces a clear error rather than silently launching with an empty map_server.

### 10.8 Random wanderer (demo mode)

`robot_navigation/scripts/random_wanderer.py` is a small ROS node that subscribes to `/map` (with `RELIABLE + TRANSIENT_LOCAL` QoS to receive the latched map), picks a random `(x, y)` cell whose `data == 0` (free) and is at least `cell_margin = 4` cells from any non-free neighbour, samples a random yaw, and sends it as a `NavigateToPose` goal. When the goal terminates (success / canceled / aborted) it waits `settle_sec = 1 s` and picks another. This gives the impression of "looking for waste" during the YOLO demo before the full mission FSM is wired in.

---

## 11. Web Dashboard

The dashboard is a stack of Node.js + Express on the backend and React on the frontend, both running on the laptop. The Pi is never a web server — it only exposes ROS topics via `rosbridge_websocket`.

### 11.1 Data flow

```
Pi ROS2 topics
    ──► rosbridge_websocket :9090   (running on Pi)
            ──► laptop Node backend  (roslib subscriber)
                    ──► MongoDB    (RobotTelemetry, every 10 s)
                    ──► HTTP polling endpoints (every 3 s from frontend)

Frontend button
    ──► HTTP POST /api/robot/*   (laptop backend)
            ──► roslib service or topic publish
                    ──► Pi
```

### 11.2 Backend (`web/backend/`)

* **`server.js`** — boots Express, validates `JWT_SECRET` + `MONGODB_URI` env vars, connects MongoDB, starts the rosbridge service + the noVNC server.
* **`services/rosBridgeService.js`** — a singleton wrapper around `roslib`. On connect:
  * Subscribes to `/odom`, `/system/{cpu_temp,cpu_usage,ram_usage,fan_speed,throttled}`, `/bins/{organic_level,non_organic_level}`.
  * Caches the latest value of each in a `latestData` object.
  * Exposes `publishCmdVel(linear, angular)` for teleop, and `callEmptyBins()` / `callShutdown()` for the buttons.
  * On disconnect: exponential back-off reconnect (`5 s → 10 → 20 → 40 → max 60 s`).
  * **Speed** is computed in the subscriber as `sqrt(vx² + vy²)` so the frontend only needs to display a number.
* **`_startSaveInterval`** — every `SAVE_INTERVAL_MS = 10 000` ms, `latestData` is written to MongoDB as a `RobotTelemetry` document. The Mongo schema has a TTL index on `timestamp` set to 7 days, so old telemetry auto-expires.
* **Auth** — JWT-based, 7-day expiry. `register` and `login` return a token; `auth` middleware (`Bearer ...`) protects `/api/robot/*` and `/api/auth/me`. Passwords are hashed with bcrypt (cost 12) inside a Mongoose `pre('save')` hook on the `User` schema.
* **`routes/robotRoutes.js`** — `POST /empty-bins`, `POST /shutdown`, `POST /teleop {linear, angular}`, `POST /stop`. Each forwards to the corresponding rosBridgeService call.
* **`routes/telemetryRoutes.js`** — `GET /latest` (live cache or DB fallback), `GET /history?limit=N` (last N records).
* **`services/novncService.js`** — spawns Xvfb on `:1`, `x11vnc` on port 5900 listening only on localhost, and `websockify` on port 6080 serving noVNC. The operator then starts RViz with `DISPLAY=:1 ros2 launch ...` and the dashboard's `<iframe src=http://localhost:6080/vnc.html?...>` displays it.

### 11.3 Frontend (`web/frontend/`)

A Vite-built React app:

* **`AuthContext`** — global auth state. On mount, if a token is in `localStorage` it calls `/api/auth/me` to validate and load the user; on 401 it clears the token.
* **`Dashboard.jsx`** — five-tab dashboard:
  1. **Overview** — speed hero card, CPU temp hero card, power-status hero card (`vcgencmd get_throttled` decoded via bitmask), speed-over-time + CPU/RAM charts.
  2. **Bins** — two SVG donut gauges, an "Empty Bins" button that POSTs to `/api/robot/empty-bins`, a "near full" warning when either > 90 %.
  3. **System** — four SVG arc gauges (CPU temp, CPU%, RAM%, fan RPM), a `vcgencmd`-decoded power-status card, and a system-history chart.
  4. **Control** — `ManualControl.jsx`, the teleop UI. On-screen D-pad + keyboard handler (WASD/arrows) sending `/api/robot/teleop` at 5 Hz; `Space` triggers an emergency-stop that POSTs `/stop` and locks the UI for 3 s. Linear / angular speed sliders bind to refs so the keyboard handler always reads the latest values without re-registering.
  5. **Tracking** — an `<iframe>` to `VITE_NOVNC_URL` (default `http://192.168.0.165:6080/vnc.html?autoconnect=true&resize=scale`) which is the noVNC stream of the laptop's RViz session.
* **`api.js`** — fetch wrapper that injects `Authorization: Bearer <token>`, handles 401 by redirecting to login, throws `"Network error - server may be offline"` on fetch failures.

### 11.4 What the dashboard talks to on the Pi

| Pi-side service           | Dashboard action                |
| ------------------------- | ------------------------------- |
| `system_monitor_node`     | All `/system/*` gauges          |
| `bin_levels_node`         | Bin donut gauges + `/bins/reset` service call (Empty Bins button) |
| `motor_bridge` (`/cmd_vel`) | Manual teleop joystick + emergency stop |
| (planned) `/robot/shutdown` Trigger service | "Shutdown Robot" button |

### 11.5 Dashboard-side helpers

* **Telemetry placeholder**: `bin_levels_node` defaults both levels to 0 and accepts `/bins/set_{organic,non_organic}` Float32 messages so the gripper FSM (when fully wired) can increment them on each pick.
* **`odom_node.py`**: a deliberately unregistered placeholder. It mirrors `/cmd_vel` into a fake `/odom` for the dashboard when the real motor bridge isn't available. The `setup.py` for `robot_dashboard` explicitly does **not** add it to `entry_points` — to prevent it being accidentally launched alongside `motor_bridge` and clobbering the real odometry.

---

## 12. Operational Procedures

### 12.1 Build

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 12.2 Map an arena

```bash
ros2 launch robot_slam slam_bringup.launch.py
# in a second terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# drive every part of the arena slowly, then in a third terminal:
ros2 run nav2_map_server map_saver_cli -f $HOME/arena_map
```

### 12.3 Autonomous run

```bash
# 1. terminal: bring up the full stack
ros2 launch robot_navigation nav_bringup.launch.py map:=$HOME/arena_map.yaml

# 2. RViz on the laptop:
ros2 run rviz2 rviz2 -d ros2_ws/src/robot_navigation/rviz/nav2.rviz
# → click "2D Pose Estimate" on the robot's real position
# → wait ~2 s for AMCL particles to converge
# → click "Nav2 Goal" anywhere to drive there

# 3. on the laptop: YOLO + mission
ros2 launch robot_vision yolo.launch.py     # laptop
ros2 launch robot_vision mission.launch.py  # Pi (separate ssh)
ros2 launch robot_vision camera.launch.py   # Pi (separate ssh)

# 4. demo wanderer (optional)
python3 ros2_ws/src/robot_navigation/scripts/random_wanderer.py
```

### 12.4 Dashboard

```bash
# Pi
ros2 launch robot_dashboard dashboard.launch.py     # spawns rosbridge_websocket on :9090

# Laptop
cd web/backend  && npm i && npm start
cd web/frontend && npm i && npm run dev
# → http://localhost:5173
```

---

## 13. Algorithm Index (one-line summaries)

| Algorithm                                | Where it lives                                            | What it does                                          |
| ---------------------------------------- | --------------------------------------------------------- | ----------------------------------------------------- |
| Differential-drive forward kinematics    | `motor_bridge.py: cmd_vel_callback`                       | `Twist → (v_L, v_R) → PWM`                            |
| Differential-drive wheel odometry        | `motor_bridge.py: _process_odometry`                      | `(Δticks_L, Δticks_R) → (Δx, Δy, Δθ)` mid-point Euler |
| Stiction breakout                        | `motor_bridge.py: _apply_stiction`                        | Floor non-zero PWM to `min_pwm` so wheels actually move |
| Bilateral command/odometry watchdog      | Pi watchdog + Arduino watchdog                            | Stop wheels if comms drop                              |
| MPU-6500 burst I²C read                  | `imu_node.py: _read_all`                                  | Single 14-byte block read = atomic IMU sample         |
| Gyro bias calibration                    | `imu_node.py: _calibrate_gyro`                            | Average 200 stationary samples to subtract bias       |
| robot_localization EKF (2-D)             | `ekf.yaml`                                                | Fuse encoder vx + encoder ω + gyro ω                  |
| Ceres scan-matching + pose-graph SLAM    | `slam_toolbox` + `slam_toolbox.yaml`                      | Build 2-D occupancy map with loop closure              |
| Likelihood-field Monte-Carlo Localisation| `nav2_amcl` + `nav2_params.yaml` `amcl` block             | 500–5000-particle MCL on saved map                     |
| NavFn Dijkstra global planner            | `nav2_planner` + `planner_server` block                   | Shortest-path on global costmap                        |
| DWB local planner + 7 critics            | `nav2_controller` + `FollowPath` block                    | Dynamic-window trajectory sampling                     |
| 2-D costmap layers (static / obstacle / inflation) | `nav2_costmap_2d` + both costmap blocks         | Build local + global costmaps from LiDAR + map        |
| Spin / backup / wait recovery behaviours | `nav2_behaviors`                                          | BT-driven recovery on path failure                     |
| YOLOv11 object detection                 | `yolo_node.py` (Ultralytics)                              | Per-frame bounding boxes + classes                     |
| IoU-based multi-frame tracker            | `temporal_filter.py: TemporalConsistencyFilter.update`    | Require N consistent frames before publishing          |
| Small-object whitelist bypass            | `yolo_node.py` (pixel-area < 1 % of frame)                | Skip temporal filter for very small real objects       |
| Mission "stop + trigger + cooldown"      | `mission_node.py: _stop_and_trigger`                      | 2-s zero-twist, 2-byte serial trigger, 10-s lock       |
| Gripper FSM (6 servos + HC-SR04)         | Arduino #2 firmware                                       | LOWER→CLOSE→LIFT→SWING→RELEASE→HOME with retry        |
| Ultrasonic-feedback descent              | Arduino #2 LOWER state                                    | Step shoulder/elbow until echo < 5 cm                  |
| 3-attempt grip-retry                     | Arduino #2 CLOSE state                                    | Open + lift + retry up to 3× before abort              |
| Bin-full mark-and-redirect               | `mission_node` + `bin_levels_node`                        | When `/bins/*` > 100 %, send Nav2 goal to dump point   |
| Random free-cell goal sampler            | `random_wanderer.py`                                      | Reject-sample random Nav2 goals from `/map`            |
| `vcgencmd get_throttled` bitmask decoder | `Dashboard.jsx: throttleInfo`                             | Translate Pi 5 power status into UI labels             |
| JWT auth + bcrypt password hashing       | `authController.js` + `User.js` Mongoose hook             | Login / register / `/me`                               |
| Exponential-backoff rosbridge reconnect  | `rosBridgeService.js: on('close')`                        | Recover from Wi-Fi blips                               |
| Latest-cache + 10-s Mongo persistence    | `rosBridgeService.js: _startSaveInterval`                 | Telemetry buffer + history endpoint                    |
| noVNC pipeline (Xvfb + x11vnc + websockify) | `novncService.js`                                       | Stream RViz from laptop display into dashboard iframe  |

---

## 14. Known limitations & design decisions

* **Two Arduinos** instead of one for separation of concerns: motor timing is hard-real-time at 20 Hz; gripper movements are slow and blocking. Putting them on one MCU would require a non-blocking servo driver and complicate the watchdog logic.
* **YOLO on the laptop**, not the Pi: Nano runs at ~12 fps on the laptop but only ~3 fps on the Pi 5 CPU. Adding a Coral / Hailo accelerator was scoped out for the FYP timeline.
* **No magnetometer fusion**: indoor mag interference (motors, steel desks) makes the AK8963 worse than useless. Yaw comes from gyro + odometry only; AMCL corrects long-term drift via the laser map.
* **No GPS**: indoor mission. The GPS-based UI was replaced with an RViz noVNC iframe.
* **Battery monitor is a stub** in the current source — the ADS1115 hardware is wired but the read code currently returns 0 until calibration is done. The schema and dashboard already render a battery bar so flipping it on is just a publisher edit.
* **Ultrasonic deliberately not in URDF**: it is only used inside the gripper FSM, not by Nav2, and adding it as a TF frame would invite the costmap to treat it as a sensor source (which it isn't).
* **Velocity smoother off**: stiction compensation already prevents lurching at takeoff. Adding the smoother is a future optimisation.

---

## 15. File index (where to find what)

```
ros2_ws/src/
├── robot_description/
│   ├── urdf/{robot.urdf.xacro,dimensions.xacro,materials.xacro,base.xacro,sensors.xacro}
│   ├── config/{slam_toolbox.yaml,nav2_params.yaml}
│   └── launch/{display.launch.py,slam.launch.py,nav2.launch.py}
├── robot_motors/
│   ├── config/hardware_params.yaml
│   ├── arduino/motor_control.ino             ← Arduino #1 firmware
│   ├── launch/motor_bridge.launch.py
│   └── robot_motors/motor_bridge.py           ← differential-drive bridge + odometry
├── robot_imu/
│   ├── config/ekf.yaml                        ← robot_localization fusion config
│   ├── launch/imu.launch.py
│   └── robot_imu/imu_node.py                  ← MPU-6500 I²C driver
├── robot_slam/
│   └── launch/slam_bringup.launch.py          ← brings motor + IMU + LiDAR + slam_toolbox
├── robot_navigation/
│   ├── launch/nav_bringup.launch.py           ← full autonomous stack
│   ├── rviz/nav2.rviz                         ← preconfigured RViz layout
│   └── scripts/random_wanderer.py
├── robot_vision/
│   ├── launch/{camera.launch.py,yolo.launch.py,mission.launch.py}
│   └── robot_vision/{yolo_node.py,temporal_filter.py,mission_node.py}
├── robot_dashboard/
│   ├── launch/dashboard.launch.py             ← rosbridge_websocket + system/bin nodes
│   └── robot_dashboard/{system_monitor_node.py,bin_levels_node.py,odom_node.py}
└── rplidar_ros/                                ← vendored Slamtec driver (unchanged)

web/
├── backend/
│   ├── server.js                              ← Express entrypoint
│   ├── services/{rosBridgeService.js,novncService.js}
│   ├── routes/{authRoutes,robotRoutes,telemetryRoutes}.js
│   ├── controllers/{authController,telemetryController}.js
│   ├── models/{User.js,RobotTelemetry.js}
│   ├── middleware/auth.js
│   └── config/db.js
└── frontend/                                  ← Vite + React
    └── src/
        ├── App.jsx
        ├── context/AuthContext.jsx
        ├── services/api.js
        └── pages/
            ├── auth/{Login,Register}.jsx
            ├── dashboard/{Dashboard,ManualControl}.jsx
            └── manage/Manage.jsx
```
