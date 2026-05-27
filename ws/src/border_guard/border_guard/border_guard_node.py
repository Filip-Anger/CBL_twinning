# Copyright 2024 falinux
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Border Guard Node — geofence that blocks motion outside a recorded polygon."""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PolygonStamped, Point32

# REBUILD UDING SLAM

class BorderGuardNode(Node):
    """ROS2 node that enforces a polygon geofence using SLAM localization."""

    def __init__(self):
        super().__init__('border_guard_node')

        # ---------- Parameters ----------
        self.declare_parameter('polygon_file', '~/border_maps/border_polygon_new.csv')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')

        polygon_path = os.path.expanduser(
            self.get_parameter('polygon_file').value
        )
        self.odom_topic = self.get_parameter('odom_topic').value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').value

        # ---------- Load polygon ----------
        self.polygon = self._load_polygon(polygon_path)
        if not self.polygon:
            self.get_logger().error(
                f'Failed to load polygon from: {polygon_path}'
            )
            self.get_logger().error(
                'Node will block ALL motion until a valid polygon is loaded.'
            )

        # ---------- State ----------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.inside_border = True
        self.position_received = False

        # ---------- Subscribers ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.input_cmd_topic,
            self._cmd_callback,
            10,
        )

        # ---------- Publishers ----------
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_cmd_topic,
            10,
        )
        self.polygon_pub = self.create_publisher(
            PolygonStamped,
            'geofence_border',
            10,
        )

        # ---------- Timer to publish geofence for RViz ----------
        self.pub_timer = self.create_timer(1.0, self._publish_geofence)

        # ---------- Startup log ----------
        self.get_logger().info('Border Guard Node started')
        self.get_logger().info(
            f'  Polygon: {polygon_path} '
            f'({len(self.polygon)} vertices)'
        )
        self.get_logger().info(
            f'  {self.input_cmd_topic} → filter → {self.output_cmd_topic}'
        )

    def _publish_geofence(self):
        """Publish the loaded geofence border as a PolygonStamped message for RViz."""
        if not self.polygon:
            return
        
        msg = PolygonStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Load the vertices
        msg.polygon.points = [
            Point32(x=float(pt[0]), y=float(pt[1]), z=0.0)
            for pt in self.polygon
        ]
        
        # Close the loop for visualization in RViz
        if len(self.polygon) > 0:
            msg.polygon.points.append(
                Point32(x=float(self.polygon[0][0]), y=float(self.polygon[0][1]), z=0.0)
            )
            
        self.polygon_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Polygon loader
    # ------------------------------------------------------------------ #
    def _load_polygon(self, filepath):
        """Load polygon vertices from a CSV file (x,y columns)."""
        polygon = []
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                header = next(reader, None)  # skip header row
                if header is None:
                    self.get_logger().error(f'Empty CSV file: {filepath}')
                    return []

                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            polygon.append((x, y))
                        except ValueError:
                            continue  # skip malformed rows

        except FileNotFoundError:
            self.get_logger().error(f'Polygon file not found: {filepath}')
            return []
        except Exception as e:
            self.get_logger().error(f'Error reading polygon file: {e}')
            return []

        # Remove closing point if present (first == last)
        if len(polygon) >= 2 and polygon[0] == polygon[-1]:
            polygon = polygon[:-1]

        if len(polygon) < 3:
            self.get_logger().error(
                f'Polygon needs at least 3 vertices, got {len(polygon)}'
            )
            return []

        self.get_logger().info(
            f'Loaded polygon with {len(polygon)} vertices'
        )
        return polygon

    # ------------------------------------------------------------------ #
    #  Odometry callback
    # ------------------------------------------------------------------ #
    def _odom_callback(self, msg: Odometry):
        """Update robot position and check if inside the polygon."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.position_received = True

        was_inside = self.inside_border
        self.inside_border = self._point_in_polygon(
            self.robot_x, self.robot_y, self.polygon
        )

        # Log transitions
        if was_inside and not self.inside_border:
            self.get_logger().warn(
                f'Robot LEFT the border at ({self.robot_x:.2f}, '
                f'{self.robot_y:.2f}) — blocking motion!'
            )
        elif not was_inside and self.inside_border:
            self.get_logger().info(
                f'Robot RETURNED inside the border at '
                f'({self.robot_x:.2f}, {self.robot_y:.2f})'
            )

    # ------------------------------------------------------------------ #
    #  Command velocity filter
    # ------------------------------------------------------------------ #
    def _cmd_callback(self, msg: TwistStamped):
        """Forward or block velocity commands based on geofence status."""
        # If no polygon loaded, block everything
        if not self.polygon:
            self._publish_stop(msg)
            return

        # If we haven't received a position yet, block for safety
        if not self.position_received:
            self._publish_stop(msg)
            return

        # If inside the border, pass through
        if self.inside_border:
            self.cmd_pub.publish(msg)
            return

        # Outside the border — block linear motion, allow rotation
        safe_cmd = TwistStamped()
        safe_cmd.header = msg.header
        safe_cmd.twist.linear.x = 0.0
        safe_cmd.twist.linear.y = 0.0
        safe_cmd.twist.linear.z = 0.0
        safe_cmd.twist.angular.x = 0.0
        safe_cmd.twist.angular.y = 0.0
        safe_cmd.twist.angular.z = msg.twist.angular.z  # allow turning

        self.get_logger().warn(
            f'GEOFENCE: Blocking motion at ({self.robot_x:.2f}, '
            f'{self.robot_y:.2f}) — outside border!',
            throttle_duration_sec=2.0,
        )
        self.cmd_pub.publish(safe_cmd)

    def _publish_stop(self, original_msg: TwistStamped):
        """Publish a zero-velocity command."""
        stop_cmd = TwistStamped()
        stop_cmd.header = original_msg.header
        self.cmd_pub.publish(stop_cmd)

    # ------------------------------------------------------------------ #
    #  Point-in-polygon (ray casting algorithm)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _point_in_polygon(px, py, polygon):
        """
        Determine if a point is inside a polygon using ray casting.

        Cast a horizontal ray from the point to the right and count
        how many polygon edges it crosses. Odd = inside, even = outside.
        """
        if not polygon:
            return False

        n = len(polygon)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            # Check if the ray crosses this edge
            if ((yi > py) != (yj > py)) and \
               (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside

            j = i

        return inside


def main(args=None):
    """Entry point for the border_guard_node."""
    rclpy.init(args=args)
    node = BorderGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
