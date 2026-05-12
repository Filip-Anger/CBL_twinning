#!/usr/bin/env python3
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, String


def min_front_range(scan: LaserScan, front_half_angle_rad: float) -> float:
    """Minimum valid range in +/- front_half_angle_rad around 0 rad."""
    if scan.angle_increment == 0.0 or not scan.ranges:
        return float("inf")

    i0 = int(round((0.0 - front_half_angle_rad - scan.angle_min) / scan.angle_increment))
    i1 = int(round((0.0 + front_half_angle_rad - scan.angle_min) / scan.angle_increment))
    i0 = max(0, min(i0, len(scan.ranges) - 1))
    i1 = max(0, min(i1, len(scan.ranges) - 1))
    if i1 < i0:
        i0, i1 = i1, i0

    m = float("inf")
    for r in scan.ranges[i0:i1 + 1]:
        if not math.isfinite(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue
        if r < m:
            m = r
    return m


class TwinSafetySupervisor(Node):
    """
    Gazebo-only DT supervisor:
      - Subscribe: /scan, /twin/cmd_vel_raw, /twin/limits/*
      - Publish:   /cmd_vel (safe), /twin/alerts
    """
    def __init__(self):
        super().__init__("twin_safety_supervisor")

        # Topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_in_topic", "/twin/cmd_vel_raw")
        self.declare_parameter("cmd_out_topic", "/cmd_vel")

        # Base policy (dry)
        self.declare_parameter("stop_distance_base", 0.50)   # meters
        self.declare_parameter("front_angle_deg", 30.0)      # +/- degrees
        self.declare_parameter("scan_timeout_s", 0.5)
        self.declare_parameter("fail_safe_on_stale_scan", True)

        # Dynamic limits (from weather_adapter)
        self.speed_scale = 1.0
        self.stop_add = 0.0

        self.min_front: float = float("inf")
        self.t_scan: Optional[float] = None

        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_in_topic = str(self.get_parameter("cmd_in_topic").value)

        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, cmd_in_topic, self.on_cmd_in, 10)

        self.create_subscription(Float32, "/twin/limits/speed_scale", self.on_speed_scale, 10)
        self.create_subscription(Float32, "/twin/limits/stop_distance_add", self.on_stop_add, 10)

        cmd_out_topic = str(self.get_parameter("cmd_out_topic").value)
        self.pub_cmd = self.create_publisher(TwistStamped, cmd_out_topic, 10)
        self.pub_alert = self.create_publisher(String, "/twin/alerts", 10)

        self.last_hazard: Optional[bool] = None

        self.get_logger().info("TwinSafetySupervisor started (Gazebo-only).")

    def on_speed_scale(self, msg: Float32):
        self.speed_scale = max(0.0, float(msg.data))

    def on_stop_add(self, msg: Float32):
        self.stop_add = max(0.0, float(msg.data))

    def on_scan(self, scan: LaserScan):
        ang = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.min_front = min_front_range(scan, ang)
        self.t_scan = time.monotonic()

    def scan_stale(self) -> bool:
        if self.t_scan is None:
            return True
        timeout = float(self.get_parameter("scan_timeout_s").value)
        return (time.monotonic() - self.t_scan) > timeout

    def on_cmd_in(self, cmd: TwistStamped):
        stop_dist = float(self.get_parameter("stop_distance_base").value) + self.stop_add

        stale = self.scan_stale()
        fail_safe = bool(self.get_parameter("fail_safe_on_stale_scan").value)

        if fail_safe and stale:
            hazard = True
            reason = "STALE_SCAN"
        else:
            hazard = (self.min_front < stop_dist)
            reason = f"min_front={self.min_front:.2f} stop_dist={stop_dist:.2f}"

        out = TwistStamped()
        out.header = cmd.header
        out.twist = cmd.twist

        if hazard:
            out.twist.linear.x = 0.0
            out.twist.linear.y = 0.0
            out.twist.linear.z = 0.0
            out.twist.angular.x = 0.0
            out.twist.angular.y = 0.0
            out.twist.angular.z = 0.0
        else:
            s = self.speed_scale
            out.twist.linear.x *= s
            out.twist.linear.y *= s
            out.twist.linear.z *= s
            out.twist.angular.z *= s

        self.pub_cmd.publish(out)

        if self.last_hazard is None or hazard != self.last_hazard:
            self.pub_alert.publish(String(
                data=f"hazard={hazard} | {reason} | speed_scale={self.speed_scale:.2f} stop_add={self.stop_add:.2f}"
            ))
            self.last_hazard = hazard


def main():
    rclpy.init()
    node = TwinSafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
