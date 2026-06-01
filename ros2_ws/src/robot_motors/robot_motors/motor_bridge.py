#!/usr/bin/env python3

# ================================================================
# GREEN GUARDIAN — Motor Bridge Node
# Package:    robot_motors
# Node name:  motor_bridge
# Author:     Mohammed Farhat
#
# Subscribes:
#   /cmd_vel  (geometry_msgs/Twist)  ← from Nav2 or teleop
#
# Publishes:
#   /odom     (nav_msgs/Odometry)    → to SLAM / Nav2
#
# Broadcasts TF:
#   odom → base_footprint
#
# Serial (Pi5 ↔ Arduino #1):
#   Send:    M:LEFT,RIGHT\n  (PWM -249 to 249)
#   Receive: O:LEFT,RIGHT\n  (encoder ticks at 20Hz)
# ================================================================
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

import serial
import math
import time
import threading


class MotorBridge(Node):

    def __init__(self):
        super().__init__('motor_bridge')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('serial_port',     '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('wheel_radius',    0.0375)
        self.declare_parameter('track_width',     0.341)
        self.declare_parameter('ticks_per_rev',   293)
        self.declare_parameter('max_pwm',         249)
        self.declare_parameter('min_pwm',         70)   # stiction breakout
        self.declare_parameter('max_velocity',    0.5)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.port         = self.get_parameter('serial_port').value
        self.baud         = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width  = self.get_parameter('track_width').value
        self.ticks_per_rev= self.get_parameter('ticks_per_rev').value
        self.max_pwm      = self.get_parameter('max_pwm').value
        self.min_pwm      = self.get_parameter('min_pwm').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.cmd_timeout  = self.get_parameter('cmd_vel_timeout').value

        # Meters traveled per encoder tick
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.get_logger().info(f'Meters per tick: {self.m_per_tick:.6f} m')

        # ── Serial connection ────────────────────────────────────
        self.serial_conn = None
        self._connect_serial()

        # ── Odometry state ───────────────────────────────────────
        self.x      = 0.0
        self.y      = 0.0
        self.theta  = 0.0
        self.left_wheel_angle  = 0.0
        self.right_wheel_angle = 0.0
        self.odom_lock       = threading.Lock()
        self._last_odom_time = None   # wall-clock time of last encoder message
        self._motors_stopped = False  # True once watchdog stops; reset by cmd_vel

        # ── QoS profiles ─────────────────────────────────────────
        # /odom must be RELIABLE so Nav2 controller_server and EKF can subscribe.
        # (controller_server logs "incompatible QoS / RELIABILITY_QOS_POLICY" and
        #  receives ZERO odometry messages when this is BEST_EFFORT.)
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # /cmd_vel — keep BEST_EFFORT; teleop and Nav2 both tolerate this.
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ── Publishers ───────────────────────────────────────────
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', odom_qos)

        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ──────────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, sensor_qos)

        # ── cmd_vel watchdog timer (10Hz check) ──────────────────
        self.last_cmd_time = time.time()
        self.create_timer(0.1, self.watchdog_callback)

        # ── Serial read thread ───────────────────────────────────
        # Runs in background — never blocks ROS2 executor
        self.running = True
        self.read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self.read_thread.start()

        self.get_logger().info('Motor bridge node started')

    # ================================================================
    # SERIAL
    # ================================================================
    def _connect_serial(self):
        """Connect to Arduino #1. Retries every 2 seconds until success."""
        while rclpy.ok():
            try:
                self.serial_conn = serial.Serial(
                    self.port, self.baud, timeout=1.0)
                time.sleep(2.0)  # wait for Arduino bootloader to finish
                self.serial_conn.reset_input_buffer()
                self.get_logger().info(
                    f'Connected to Arduino on {self.port} at {self.baud} baud')
                return
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Cannot open {self.port}: {e}  — retrying in 2s')
                time.sleep(2.0)

    def _send_motor_command(self, left_pwm: int, right_pwm: int):
        """Send M:LEFT,RIGHT\\n to Arduino."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        try:
            cmd = f'M:{left_pwm},{right_pwm}\n'
            self.serial_conn.write(cmd.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _serial_read_loop(self):
        """Background thread — reads lines from Arduino continuously."""
        while self.running and rclpy.ok():
            if self.serial_conn is None or not self.serial_conn.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = self.serial_conn.readline()
                line = raw.decode('utf-8', errors='ignore').strip()

                if line.startswith('O:'):
                    self._process_odometry(line)
                elif line.startswith('GG:'):
                    self.get_logger().info(f'Arduino says: {line}')
                elif line.startswith('E:'):
                    self.get_logger().warn(f'Arduino error: {line}')
                # OK lines are intentionally ignored — no need to log them

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e} — attempting reconnect')
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None
                self._connect_serial()
            except Exception as e:
                self.get_logger().error(f'Unexpected read error: {e}')

    # ================================================================
    # CMD_VEL CALLBACK
    # Differential drive kinematics: Twist → left/right PWM
    # ================================================================
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time   = time.time()
        self._motors_stopped = False

        linear  = msg.linear.x    # m/s  (+forward, -backward)
        angular = msg.angular.z   # rad/s (+left, -right)

        # Differential drive wheel velocity equations:
        #   v_left  = linear - angular * (track_width / 2)
        #   v_right = linear + angular * (track_width / 2)
        v_left  = linear - (angular * self.track_width / 2.0)
        v_right = linear + (angular * self.track_width / 2.0)

        # Scale velocity to PWM
        # max_velocity (m/s) maps to max_pwm
        # ⚠️ Tune max_velocity in hardware_params.yaml after ground testing
        scale = self.max_pwm / self.max_velocity

        pwm_left  = int(v_left  * scale)
        pwm_right = int(v_right * scale)

        # Stiction compensation: motors stall below ~PWM 60-70 on the ground
        # (gearbox + cable drag). Bump any non-zero command up to min_pwm so
        # Nav2's small low-speed rotation commands actually move the wheels.
        pwm_left  = self._apply_stiction(pwm_left)
        pwm_right = self._apply_stiction(pwm_right)

        # Hard clamp — never exceed max_pwm
        pwm_left  = max(-self.max_pwm, min(self.max_pwm, pwm_left))
        pwm_right = max(-self.max_pwm, min(self.max_pwm, pwm_right))

        self._send_motor_command(pwm_left, pwm_right)

    def _apply_stiction(self, pwm: int) -> int:
        if pwm == 0:
            return 0
        if abs(pwm) < self.min_pwm:
            return self.min_pwm if pwm > 0 else -self.min_pwm
        return pwm

    # ================================================================
    # WATCHDOG
    # ================================================================
    def watchdog_callback(self):
        """Stop motors if no /cmd_vel received within timeout."""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if not self._motors_stopped:
                self._send_motor_command(0, 0)
                self._motors_stopped = True

    # ================================================================
    # ODOMETRY
    # ================================================================
    def _process_odometry(self, line: str):
        """
        Parse O:LEFT,RIGHT from Arduino.
        Update pose using differential drive odometry equations.
        Publish /odom and broadcast odom→base_footprint TF.
        """
        try:
            data = line[2:].split(',')
            if len(data) != 2:
                return
            left_ticks  = int(data[0])
            right_ticks = int(data[1])
        except ValueError:
            self.get_logger().warn(f'Malformed odometry line: {line}')
            return

        # Measure actual elapsed time since last encoder message
        now_wall = time.time()
        if self._last_odom_time is None or (now_wall - self._last_odom_time) <= 0.0:
            dt = 0.05   # safe fallback for the very first message
        else:
            dt = now_wall - self._last_odom_time
        self._last_odom_time = now_wall

        # Convert ticks to meters
        d_left  = left_ticks  * self.m_per_tick
        d_right = right_ticks * self.m_per_tick

        # Accumulate wheel rotation angles (rad) for /joint_states
        self.left_wheel_angle  += d_left  / self.wheel_radius
        self.right_wheel_angle += d_right / self.wheel_radius

        # Differential drive pose update:
        #   d_center = average distance traveled
        #   d_theta  = rotation from wheel speed difference
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.track_width

        with self.odom_lock:
            # Integrate position using midpoint angle to minimise heading error
            mid_theta   = self.theta + d_theta / 2.0
            self.theta += d_theta
            self.x     += d_center * math.cos(mid_theta)
            self.y     += d_center * math.sin(mid_theta)

            # Keep theta in [-pi, pi]
            self.theta = math.atan2(
                math.sin(self.theta), math.cos(self.theta))

            x, y, theta = self.x, self.y, self.theta

        now = self.get_clock().now().to_msg()

        # Quaternion from yaw angle (rotation around Z only)
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # ── Publish /odom ────────────────────────────────────────
        odom                         = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x    = d_center / dt
        odom.twist.twist.angular.z   = d_theta  / dt

        # Pose covariance [x, y, z, roll, pitch, yaw] diagonal
        odom.pose.covariance[0]  = 0.01   # x variance
        odom.pose.covariance[7]  = 0.01   # y variance
        odom.pose.covariance[35] = 0.05   # yaw variance

        # Twist covariance
        odom.twist.covariance[0]  = 0.01  # linear.x variance
        odom.twist.covariance[35] = 0.05  # angular.z variance

        self.odom_pub.publish(odom)

        # ── Publish /joint_states (wheel angles for robot_state_publisher) ──
        js = JointState()
        js.header.stamp = now
        js.name     = ['wheel_front_left_joint', 'wheel_front_right_joint',
                        'wheel_rear_left_joint',  'wheel_rear_right_joint']
        js.position = [self.left_wheel_angle,  self.right_wheel_angle,
                       self.left_wheel_angle,  self.right_wheel_angle]
        self.joint_state_pub.publish(js)

        # ── Broadcast TF: odom → base_footprint ─────────────────
        # ── Broadcast TF: odom → base_footprint ─────────────────
        if self.publish_tf:
            tf                          = TransformStamped()
            tf.header.stamp             = now
            tf.header.frame_id          = 'odom'
            tf.child_frame_id           = 'base_footprint'
            tf.transform.translation.x  = x
            tf.transform.translation.y  = y
            tf.transform.translation.z  = 0.0
            tf.transform.rotation.x     = 0.0
            tf.transform.rotation.y     = 0.0
            tf.transform.rotation.z     = qz
            tf.transform.rotation.w     = qw
            self.tf_broadcaster.sendTransform(tf)

    # ================================================================
    # CLEANUP
    # ================================================================
    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self._send_motor_command(0, 0)  # safety stop
            time.sleep(0.1)
            self.serial_conn.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


# ================================================================
# ENTRY POINT
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
