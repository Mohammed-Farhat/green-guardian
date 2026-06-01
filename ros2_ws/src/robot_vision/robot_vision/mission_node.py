#!/usr/bin/env python3
# ================================================================
# GREEN GUARDIAN — Mission Node
# Package:  robot_vision
# Node:     mission_node  (runs on Pi)
#
# Subscribes:
#   /garbage_detection  (std_msgs/String)  "organic" | "non_organic" | "none"
#
# Publishes:
#   /cmd_vel  (geometry_msgs/Twist)  — zero velocity to stop robot
#
# On detection:
#   1. Publishes zero Twist to /cmd_vel for 2 seconds.
#   2. Opens gripper_serial_port at 9600 baud.
#   3. Writes b'G' (trigger) then b'O' or b'N' (bin selector).
#   4. Enters cooldown for cooldown_seconds, ignoring further detections.
# ================================================================
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

_STOP_DURATION = 2.0  # seconds of zero cmd_vel before triggering gripper


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('gripper_serial_port', '/dev/ttyUSB1')
        self.declare_parameter('cooldown_seconds', 10.0)

        self.gripper_port = self.get_parameter('gripper_serial_port').value
        self.cooldown = float(self.get_parameter('cooldown_seconds').value)

        # ── State ────────────────────────────────────────────────
        self._in_cooldown = False
        self._state_lock = threading.Lock()

        # ── Publisher ────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Subscriber ───────────────────────────────────────────
        self.create_subscription(
            String, '/garbage_detection', self._detection_callback, 10)

        self.get_logger().info(
            f'Mission node ready. Gripper port: {self.gripper_port}  '
            f'Cooldown: {self.cooldown}s')

    # ================================================================
    # DETECTION CALLBACK
    # ================================================================
    def _detection_callback(self, msg: String):
        label = msg.data

        if label not in ('organic', 'non_organic'):
            return

        with self._state_lock:
            if self._in_cooldown:
                return
            self._in_cooldown = True

        self.get_logger().info(
            f'Detection accepted: {label} — stopping robot and triggering gripper')

        # Offload the blocking stop+trigger sequence to a daemon thread so the
        # ROS executor is never blocked.
        t = threading.Thread(
            target=self._stop_and_trigger,
            args=(label,),
            daemon=True,
        )
        t.start()

    # ================================================================
    # STOP → TRIGGER → COOLDOWN  (runs in a background thread)
    # ================================================================
    def _stop_and_trigger(self, label: str):
        # ── Phase 1: publish zero cmd_vel for STOP_DURATION seconds ──
        self.get_logger().info(
            f'Publishing zero /cmd_vel for {_STOP_DURATION}s')
        stop_msg = Twist()  # all fields default to 0.0
        deadline = time.time() + _STOP_DURATION
        while time.time() < deadline:
            self.cmd_pub.publish(stop_msg)
            time.sleep(0.1)

        # ── Phase 2: send serial signal to Arduino #2 ─────────────
        bin_byte = b'O' if label == 'organic' else b'N'
        self.get_logger().info(
            f'Sending gripper signal: G + {bin_byte!r} on {self.gripper_port}')
        try:
            with serial.Serial(self.gripper_port, 9600, timeout=1.0) as ser:
                ser.write(b'G')
                ser.write(bin_byte)
                self.get_logger().info('Gripper signal sent successfully')
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial error on {self.gripper_port}: {exc}')

        # ── Phase 3: cooldown ──────────────────────────────────────
        self.get_logger().info(
            f'Entering {self.cooldown}s cooldown — detections suppressed')
        time.sleep(self.cooldown)

        with self._state_lock:
            self._in_cooldown = False

        self.get_logger().info('Cooldown complete — ready for next detection')


# ================================================================
# ENTRY POINT
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
