#!/usr/bin/env python3
 
import math
from typing import List
 
import rclpy
from rclpy.node import Node
 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy
 
 
class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
 
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('front_angle_deg', 30.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.input_cmd_topic = self.get_parameter('input_cmd_topic').get_parameter_value().string_value
        self.output_cmd_topic = self.get_parameter('output_cmd_topic').get_parameter_value().string_value
        self.stop_distance = self.get_parameter('stop_distance').get_parameter_value().double_value
        self.front_angle_deg = self.get_parameter('front_angle_deg').get_parameter_value().double_value
 
        self.wall_detected = False
        self.latest_min_front_distance = float('inf')
 

        qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.scan_sub = self.create_subscription(
        LaserScan,
        self.scan_topic,
        self.scan_callback,
        qos_profile
        )
 
        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.input_cmd_topic,
            self.cmd_callback,
            10
        )
 
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            self.output_cmd_topic,
            10
        )
 
        self.get_logger().info('Safety Stop Node started')
 
    def scan_callback(self, msg: LaserScan):
        front_distances = self.get_front_arc_distances(msg, self.front_angle_deg)
 
        valid_ranges = [
            r for r in front_distances
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]
 
        if valid_ranges:
            self.latest_min_front_distance = min(valid_ranges)
            self.wall_detected = self.latest_min_front_distance < self.stop_distance
        else:
            self.latest_min_front_distance = float('inf')
            self.wall_detected = False
 
    def cmd_callback(self, msg: TwistStamped):
        safe_cmd = TwistStamped()
        safe_cmd.header = msg.header
 
        forward_requested = msg.twist.linear.x > 0.0
 
        if self.wall_detected and forward_requested:
            safe_cmd.twist.linear.x = 0.0
            safe_cmd.twist.linear.y = 0.0
            safe_cmd.twist.linear.z = 0.0
            safe_cmd.twist.angular.x = 0.0
            safe_cmd.twist.angular.y = 0.0
            safe_cmd.twist.angular.z = msg.twist.angular.z
 
            self.get_logger().warn(
                f'Wall detected at {self.latest_min_front_distance:.2f} m. Blocking forward motion.'
            )
        else:
            safe_cmd = msg
 
        self.cmd_pub.publish(safe_cmd)
 
    def get_front_arc_distances(self, scan_msg: LaserScan, front_angle_deg: float) -> List[float]:
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
 
        front_angle_rad = math.radians(front_angle_deg)
        selected = []
 
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
 
            if -front_angle_rad <= angle <= front_angle_rad:
                selected.append(distance)
 
        return selected
 
 
def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
