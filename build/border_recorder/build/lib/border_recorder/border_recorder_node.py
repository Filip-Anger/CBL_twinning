# Copyright 2024 falinux
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Border Recorder Node — records SLAM path via TF and exports as polygon."""

import json
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, LookupException
from tf2_ros import ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PolygonStamped, Point32


class BorderRecorderNode(Node):
    """ROS2 node that records the robot's driven path and exports it as a polygon."""

    def __init__(self):
        super().__init__('border_recorder_node')

        # ---------- Parameters ----------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('min_distance', 0.1)        # metres between recorded points
        self.declare_parameter('output_dir', '~/border_maps')
        self.declare_parameter('simplify_tolerance', 0.05)  # RDP tolerance in metres
        self.declare_parameter('tf_poll_rate', 10.0)        # Hz — how often to check TF

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_distance = self.get_parameter('min_distance').value
        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        self.simplify_tolerance = self.get_parameter('simplify_tolerance').value
        tf_poll_rate = self.get_parameter('tf_poll_rate').value

        # ---------- State ----------
        self.path_points = []   # list of (x, y) tuples
        self.last_x = None
        self.last_y = None
        self.recording = True
        self._tf_available = False  # track whether we've seen TF yet

        # ---------- Publishers for RViz ----------
        self.polygon_pub = self.create_publisher(
            PolygonStamped, 'recorded_border', 10
        )
        self.simplified_polygon_pub = self.create_publisher(
            PolygonStamped, 'recorded_border_simplified', 10
        )

        # ---------- TF Listener ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- Timer (polls TF at tf_poll_rate Hz) ----------
        timer_period = 1.0 / tf_poll_rate
        self.tf_timer = self.create_timer(timer_period, self._tf_timer_callback)

        # ---------- Services ----------
        self.save_srv = self.create_service(
            Trigger, 'save_border', self._save_border_callback
        )
        self.clear_srv = self.create_service(
            Trigger, 'clear_border', self._clear_border_callback
        )

        # ---------- Startup log ----------
        self.get_logger().info(
            f'Border Recorder started — using TF: '
            f'{self.map_frame} → {self.robot_frame} @ {tf_poll_rate} Hz'
        )
        self.get_logger().info(
            f'  min_distance={self.min_distance} m, '
            f'simplify_tolerance={self.simplify_tolerance} m'
        )
        self.get_logger().info(
            f'  output_dir={self.output_dir}'
        )
        self.get_logger().info(
            '  Services: /save_border  /clear_border'
        )

    # ------------------------------------------------------------------ #
    #  TF timer callback
    # ------------------------------------------------------------------ #
    def _tf_timer_callback(self):
        """Poll TF for the robot position and record if moved far enough."""
        if not self.recording:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),            # latest available
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._tf_available:
                self.get_logger().info(
                    f'Waiting for TF: {self.map_frame} → {self.robot_frame} '
                    '(is SLAM running?)',
                    throttle_duration_sec=5.0,
                )
            return

        if not self._tf_available:
            self._tf_available = True
            self.get_logger().info(
                f'TF available! Recording {self.map_frame} → {self.robot_frame}'
            )

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        if self.last_x is None:
            # First point — always record
            self._record_point(x, y)
            return

        dist = math.hypot(x - self.last_x, y - self.last_y)
        if dist >= self.min_distance:
            self._record_point(x, y)

    def _record_point(self, x: float, y: float):
        """Append a point, log progress, and publish."""
        self.path_points.append((x, y))
        self.last_x = x
        self.last_y = y
        if len(self.path_points) % 50 == 0:
            self.get_logger().info(
                f'Recorded {len(self.path_points)} border points'
            )
        self._publish_polygon()

    def _publish_polygon(self):
        """Publish the current recorded path as a PolygonStamped message."""
        if not self.path_points:
            return
        msg = PolygonStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [
            Point32(x=float(pt[0]), y=float(pt[1]), z=0.0)
            for pt in self.path_points
        ]
        self.polygon_pub.publish(msg)

    def _publish_simplified_polygon(self, points):
        """Publish the simplified and closed polygon."""
        msg = PolygonStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.polygon.points = [
            Point32(x=float(pt[0]), y=float(pt[1]), z=0.0)
            for pt in points
        ]
        self.simplified_polygon_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  /save_border service
    # ------------------------------------------------------------------ #
    def _save_border_callback(self, request, response):
        """Simplify the path, close the polygon, and write output files."""
        n_raw = len(self.path_points)
        if n_raw < 3:
            response.success = False
            response.message = (
                f'Need at least 3 points to form a polygon (have {n_raw}). '
                'Drive the robot further.'
            )
            self.get_logger().warn(response.message)
            return response

        # 1. Simplify with Ramer-Douglas-Peucker
        simplified = self._rdp_simplify(
            self.path_points, self.simplify_tolerance
        )

        # Make sure we still have a valid polygon after simplification
        if len(simplified) < 3:
            simplified = self.path_points  # fall back to raw

        # 2. Close the polygon (first == last)
        if simplified[0] != simplified[-1]:
            simplified.append(simplified[0])

        # 3. Ensure output directory exists
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 4. Write GeoJSON
        geojson_path = os.path.join(
            self.output_dir, f'border_polygon_{timestamp}.geojson'
        )
        self._write_geojson(simplified, geojson_path)

        # 5. Write CSV
        csv_path = os.path.join(
            self.output_dir, f'border_polygon_{timestamp}.csv'
        )
        self._write_csv(simplified, csv_path)

        # 6. Publish simplified polygon for RViz visualization
        self._publish_simplified_polygon(simplified)

        msg = (
            f'Saved border polygon: {n_raw} raw → '
            f'{len(simplified)} simplified points\n'
            f'  GeoJSON: {geojson_path}\n'
            f'  CSV:     {csv_path}'
        )
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    # ------------------------------------------------------------------ #
    #  /clear_border service
    # ------------------------------------------------------------------ #
    def _clear_border_callback(self, request, response):
        """Clear the recorded path to start over."""
        n = len(self.path_points)
        self.path_points.clear()
        self.last_x = None
        self.last_y = None
        # Publish empty polygon to clear visualizer
        self._publish_polygon()
        msg = f'Cleared {n} recorded border points.'
        self.get_logger().info(msg)
        response.success = True
        response.message = msg
        return response

    # ------------------------------------------------------------------ #
    #  Output writers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _write_geojson(points, filepath):
        """Write a GeoJSON Polygon feature."""
        # GeoJSON uses [longitude, latitude] = [x, y] for local coords
        coordinates = [[x, y] for x, y in points]
        geojson = {
            'type': 'FeatureCollection',
            'features': [
                {
                    'type': 'Feature',
                    'properties': {
                        'name': 'border_polygon',
                        'description': 'Allowed area boundary recorded by border_recorder',
                        'recorded_at': datetime.now().isoformat(),
                        'num_vertices': len(points) - 1,  # -1 for closing point
                    },
                    'geometry': {
                        'type': 'Polygon',
                        'coordinates': [coordinates],
                    },
                }
            ],
        }
        with open(filepath, 'w') as f:
            json.dump(geojson, f, indent=2)

    @staticmethod
    def _write_csv(points, filepath):
        """Write a simple CSV with x,y columns."""
        with open(filepath, 'w') as f:
            f.write('x,y\n')
            for x, y in points:
                f.write(f'{x:.6f},{y:.6f}\n')

    # ------------------------------------------------------------------ #
    #  Ramer-Douglas-Peucker path simplification
    # ------------------------------------------------------------------ #
    @staticmethod
    def _rdp_simplify(points, tolerance):
        """
        Simplify a polyline using the Ramer-Douglas-Peucker algorithm.

        Reduces the number of points while preserving the overall shape.
        """
        if len(points) <= 2:
            return list(points)

        # Find the point farthest from the line (first → last)
        start = points[0]
        end = points[-1]
        max_dist = 0.0
        max_idx = 0

        for i in range(1, len(points) - 1):
            dist = BorderRecorderNode._point_line_distance(
                points[i], start, end
            )
            if dist > max_dist:
                max_dist = dist
                max_idx = i

        # If the farthest point exceeds tolerance, recursively simplify
        if max_dist > tolerance:
            left = BorderRecorderNode._rdp_simplify(
                points[:max_idx + 1], tolerance
            )
            right = BorderRecorderNode._rdp_simplify(
                points[max_idx:], tolerance
            )
            return left[:-1] + right
        else:
            return [start, end]

    @staticmethod
    def _point_line_distance(point, line_start, line_end):
        """Perpendicular distance from a point to a line segment."""
        px, py = point
        sx, sy = line_start
        ex, ey = line_end

        dx = ex - sx
        dy = ey - sy
        line_len_sq = dx * dx + dy * dy

        if line_len_sq == 0.0:
            # start == end → distance to that point
            return math.hypot(px - sx, py - sy)

        # Project point onto the line
        t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / line_len_sq))
        proj_x = sx + t * dx
        proj_y = sy + t * dy
        return math.hypot(px - proj_x, py - proj_y)


def main(args=None):
    """Entry point for the border_recorder_node."""
    rclpy.init(args=args)
    node = BorderRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
