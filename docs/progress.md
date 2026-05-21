# Green Guardian — Progress

_Last updated: **2026-05-07**._

## Strategy

Finish the **autonomous driving stack** end-to-end (SLAM → AMCL → Nav2) before touching the gripper or the camera. Whole-system integration comes last.

```
[ drive base ✅ ]
        │
        ▼
[ IMU + EKF tuning  ◀─ current focus ]
        │
        ▼
[ SLAM mapping ]  →  [ AMCL localization ]  →  [ Nav2 autonomous goals ]
        │
        ▼
[ gripper bring-up ]  →  [ camera + vision ]  →  [ full mission FSM ]
```

## Done

- **Drive base wired up.** All 4 JGA25-370 motors connected to the DHB-1 driver and Arduino #1. **Only the two front-wheel encoders are wired** (rear wheels are mechanically slaved to the front pair, so odometry comes from the front encoders).
- **Arduino #1 firmware flashed** with `arduino/motor_control.ino` (115200 baud, `M:` / `O:` protocol, 500 ms watchdog).
- **`motor_bridge` node verified** end-to-end: `/cmd_vel` → PWM → motors, encoder ticks → `/odom` + `odom→base_footprint` TF.
- **Teleop tested** from the laptop with `teleop_twist_keyboard` driving the robot over the network.
- **RPLidar A1 connected** to the Pi 5, driver launched (`rplidar_a1_launch.py`), `/scan` verified.
- **GY-91 IMU connected** (I²C bus 1, address `0x68`); `imu_node` publishes `/imu` at 50 Hz.

## In progress

- **IMU mounting & tuning.** The GY-91 is currently hanging off the Pi 5 — not yet bolted to the chassis at the URDF-defined `imu_link` pose (0, 0, 0.0345). Tuning (bias removal, covariances, alignment with the chassis frame) is **blocked on mechanical mounting** because rotation/translation of a loose IMU corrupts any bias estimate.

## Blocked / known issues

- **IMU not fixed to chassis** — see above. First action of the next session.
- **Rear-wheel encoders unwired** — acceptable for the current chained-drive design; revisit only if differential slip becomes a problem during turns.
- **`max_velocity` (`hardware_params.yaml`) still 0.5 m/s placeholder** — needs a measured-distance / time test on the floor under battery power.

## Next up (in order)

1. **Mount the IMU** on the chassis top plate at the URDF location.
2. **Tune the IMU** — capture stationary bias, sanity-check axes (X forward, Y left, Z up), confirm `imu0_remove_gravitational_acceleration: true` is producing clean planar accel.
3. **Verify the EKF** — `/odometry/filtered` should track straight-line and pure-rotation tests better than raw `/odom`.
4. **Calibrate `max_velocity`** with a tape-measure straight-line run.
5. **Run SLAM** (`robot_description slam.launch.py`) while teleoperating the robot through the test arena; save the resulting map with `nav2_map_server map_saver_cli`.
6. **Localize with AMCL** on the saved map (`nav2.launch.py map:=…`); use the RViz "2D Pose Estimate" to seed.
7. **Send Nav2 goals** in RViz; tune costmap inflation, DWB critic scales, and AMCL `transform_tolerance` until autonomous navigation is reliable.
8. **Then** start gripper bring-up (Arduino #2 FSM and side-channel signaling).
9. **Then** camera + vision pipeline.
10. **Finally** the whole-mission integration (wander → detect → approach → pick → drop).

## Out of scope until the autonomous stack is signed off

- 6-DOF arm + gripper (Arduino #2 FSM)
- Logitech C920 driver and any vision/classification work
- Whole-robot mission state machine
