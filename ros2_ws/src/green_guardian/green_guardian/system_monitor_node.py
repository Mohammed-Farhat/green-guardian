#!/usr/bin/env python3
"""
system_monitor_node.py
Publishes CPU temperature, CPU usage, RAM usage, and fan speed at 1 Hz.

Topics published:
  /system/cpu_temp    (std_msgs/Float32) — degrees Celsius
  /system/cpu_usage   (std_msgs/Float32) — percent 0–100
  /system/ram_usage   (std_msgs/Float32) — percent 0–100
  /system/fan_speed   (std_msgs/Float32) — RPM (0 if unreadable)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import psutil
import os
import glob


class SystemMonitorNode(Node):

    def __init__(self):
        super().__init__('system_monitor_node')

        self.pub_cpu_temp  = self.create_publisher(Float32, '/system/cpu_temp',  10)
        self.pub_cpu_usage = self.create_publisher(Float32, '/system/cpu_usage', 10)
        self.pub_ram_usage = self.create_publisher(Float32, '/system/ram_usage', 10)
        self.pub_fan_speed = self.create_publisher(Float32, '/system/fan_speed', 10)

        self.timer = self.create_timer(1.0, self.publish_all)
        self.get_logger().info('SystemMonitorNode started.')

    # ------------------------------------------------------------------
    def publish_all(self):
        self.pub_cpu_temp.publish(Float32(data=self._cpu_temp()))
        self.pub_cpu_usage.publish(Float32(data=self._cpu_usage()))
        self.pub_ram_usage.publish(Float32(data=self._ram_usage()))
        self.pub_fan_speed.publish(Float32(data=self._fan_speed()))

    # ------------------------------------------------------------------
    def _cpu_temp(self) -> float:
        """Read CPU temp from thermal zone (works on Pi 5)."""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            return 0.0

    def _cpu_usage(self) -> float:
        """Overall CPU usage percent."""
        return psutil.cpu_percent(interval=None)

    def _ram_usage(self) -> float:
        """RAM usage percent."""
        return psutil.virtual_memory().percent

    def _fan_speed(self) -> float:
        """
        Pi 5 active cooler fan RPM.
        Tries hwmon fan1_input first, then falls back to cooling_device cur_state.
        Returns 0.0 if neither is readable.
        """
        # Method 1: hwmon (most accurate — gives real RPM)
        for path in glob.glob('/sys/class/hwmon/hwmon*/fan1_input'):
            try:
                with open(path) as f:
                    return float(f.read().strip())
            except Exception:
                continue

        # Method 2: cooling device state (0=off, 1–5 = speed level)
        for path in glob.glob('/sys/class/thermal/cooling_device*/cur_state'):
            try:
                with open(path) as f:
                    state = int(f.read().strip())
                    # Map 0–5 state to approximate RPM for display
                    return float(state * 1000)
            except Exception:
                continue

        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
