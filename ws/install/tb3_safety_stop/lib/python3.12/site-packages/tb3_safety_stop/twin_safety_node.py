#!/usr/bin/env python3

import math
from typing import List
#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class TwinSafetyNode(Node):
    def __init__(self):
       super().__init__('twin_safety_node_back')

       self.declare_parameter('real_scan_topic', '/scan')
       self.declare_parameter('sim_scan_topic', '/sim/scan')
       self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
       self.declare_parameter('real_cmd_topic', '/cmd_vel')
       self.declare_parameter('sim_cmd_topic', '/sim/cmd_vel')
       self.declare_parameter('stop_distance', 0.35)
       self.declare_parameter('stop_distance_back', 0.2)
       self.declare_parameter('front_angle_deg', 30.0)

       self.real_scan_topic = self.get_parameter('real_scan_topic').value
       self.sim_scan_topic = self.get_parameter('sim_scan_topic').value
       self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
       self.real_cmd_topic = self.get_parameter('real_cmd_topic').value
       self.sim_cmd_topic = self.get_parameter('sim_cmd_topic').value
       self.stop_distance = float(self.get_parameter('stop_distance').value)
       self.stop_distance_back = float(self.get_parameter('stop_distance_back').value)
       self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

       self.real_blocked_front = False
       self.sim_blocked_front = False
       self.real_min_distance_front = float('inf')
       self.sim_min_distance_front = float('inf')
       
       self.real_blocked_back = False
       self.sim_blocked_back = False
       self.real_min_distance_back = float('inf')
       self.sim_min_distance_back = float('inf')

       scan_qos = QoSProfile(
           depth=10,
           reliability=ReliabilityPolicy.BEST_EFFORT
       )

       self.create_subscription(
           LaserScan,
           self.real_scan_topic,
           self.real_scan_cb,
           scan_qos
       )

       self.create_subscription(
           LaserScan,
           self.sim_scan_topic,
           self.sim_scan_cb,
           scan_qos
       )

       self.create_subscription(
           TwistStamped,
           self.input_cmd_topic,
           self.cmd_cb,
           10
       )

       self.real_pub = self.create_publisher(TwistStamped, self.real_cmd_topic, 10)
       self.sim_pub = self.create_publisher(TwistStamped, self.sim_cmd_topic, 10)

       self.get_logger().info("Twin Safety BACK AND FRONT Node started")

    def real_scan_cb(self, msg):
       self.real_min_distance_front, self.real_blocked_front, self.real_min_distance_back, self.real_blocked_back = self.evaluate_obstacle(msg)

    def sim_scan_cb(self, msg):
       self.sim_min_distance_front, self.sim_blocked_front, self.sim_min_distance_back, self.sim_blocked_back = self.evaluate_obstacle(msg)

    def evaluate_obstacle(self, msg):
        front_ranges, back_ranges = self.get_front_arc_distances(msg, self.front_angle_deg)
        
        valid_front = [
            r for r in front_ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if not valid_front:
            min_distance_front, blocked_front = float('inf'), False
        
        min_distance_front = min(valid_front)
        blocked_front = min_distance_front < self.stop_distance
        
        
        valid_back = [
            r for r in back_ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if not valid_back:
            min_distance_back, blocked_back = float('inf'), False
        
        min_distance_back = min(valid_back)
        blocked_back = min_distance_back < self.stop_distance_back
        
        
        
        return min_distance_front, blocked_front, min_distance_back, blocked_back

    def get_front_arc_distances(self, scan_msg: LaserScan, front_angle_deg: float):
       ranges = scan_msg.ranges
       angle_min = scan_msg.angle_min
       angle_increment = scan_msg.angle_increment

       front_angle_rad = math.radians(front_angle_deg)
       back_angle_rad = math.radians(180 - front_angle_deg)
       front_selected = []
       back_selected = []
       
       for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment

            # Normalize angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Only front sector
            if abs(angle) <= front_angle_rad:
               front_selected.append(distance)
            if abs(angle) >= back_angle_rad:
                back_selected.append(distance)

       return front_selected, back_selected # List of floats (distances in front and back)

    def cmd_cb(self, msg):
        safe = TwistStamped()
        safe.header = msg.header

        blocked_front = self.real_blocked_front or self.sim_blocked_front
        forward_requested = msg.twist.linear.x > 0.0
        
        blocked_back = self.real_blocked_back or self.sim_blocked_back
        backward_requested = msg.twist.linear.x < 0.0

        self.get_logger().info(
           f"real_blocked_front={self.real_blocked_front} sim_blocked_front={self.sim_blocked_front} "
           f"real_min={self.real_min_distance_front:.2f} sim_min={self.sim_min_distance_front:.2f} "
           f"lin.x={msg.twist.linear.x:.2f} ang.z={msg.twist.angular.z:.2f}"
       )
        
        if (blocked_front and forward_requested) or (blocked_back and backward_requested):
           safe.twist.linear.x = 0.0
           safe.twist.linear.y = 0.0
           safe.twist.linear.z = 0.0
           safe.twist.angular.x = 0.0
           safe.twist.angular.y = 0.0

           # allow turning
           safe.twist.angular.z = msg.twist.angular.z

           self.get_logger().warn(f"STOP: obstacle in FRONT={blocked_front and forward_requested} or BACK ={blocked_back and backward_requested}")

        else:
            safe = msg

        self.real_pub.publish(safe)
        self.sim_pub.publish(safe)


def main():
   rclpy.init()
   node = TwinSafetyNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == "__main__":
   main()
