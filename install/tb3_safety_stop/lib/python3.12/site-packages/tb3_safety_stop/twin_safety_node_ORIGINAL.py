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
       super().__init__('twin_safety_node')

       self.declare_parameter('real_scan_topic', '/scan')
       self.declare_parameter('sim_scan_topic', '/sim/scan')
       self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
       self.declare_parameter('real_cmd_topic', '/cmd_vel')
       self.declare_parameter('sim_cmd_topic', '/sim/cmd_vel')
       self.declare_parameter('stop_distance', 0.35)
       self.declare_parameter('front_angle_deg', 30.0)

       self.real_scan_topic = self.get_parameter('real_scan_topic').value
       self.sim_scan_topic = self.get_parameter('sim_scan_topic').value
       self.input_cmd_topic = self.get_parameter('input_cmd_topic').value
       self.real_cmd_topic = self.get_parameter('real_cmd_topic').value
       self.sim_cmd_topic = self.get_parameter('sim_cmd_topic').value
       self.stop_distance = float(self.get_parameter('stop_distance').value)
       self.front_angle_deg = float(self.get_parameter('front_angle_deg').value)

       self.real_blocked = False
       self.sim_blocked = False
       self.real_min_distance = float('inf')
       self.sim_min_distance = float('inf')

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

       self.get_logger().info("Twin Safety Node started")

   def real_scan_cb(self, msg):
       self.real_min_distance, self.real_blocked = self.evaluate_front_obstacle(msg) # Float, bool

   def sim_scan_cb(self, msg):
       self.sim_min_distance, self.sim_blocked = self.evaluate_front_obstacle(msg) # Float, bool

   def evaluate_front_obstacle(self, msg):
       front_ranges = self.get_front_arc_distances(msg, self.front_angle_deg)

       valid = [
           r for r in front_ranges
           if math.isfinite(r) and msg.range_min < r < msg.range_max
       ]

       if not valid:
           return float('inf'), False

       min_distance = min(valid)
       blocked = min_distance < self.stop_distance
       return min_distance, blocked

   def get_front_arc_distances(self, scan_msg: LaserScan, front_angle_deg: float) -> List[float]:
       ranges = scan_msg.ranges
       angle_min = scan_msg.angle_min
       angle_increment = scan_msg.angle_increment

       front_angle_rad = math.radians(front_angle_deg)
       selected = []

       for i, distance in enumerate(ranges):
           angle = angle_min + i * angle_increment

           # Normalize angle to [-pi, pi]
           angle = math.atan2(math.sin(angle), math.cos(angle))

           # Only front sector
           if abs(angle) <= front_angle_rad:
               selected.append(distance)

       return selected

   def cmd_cb(self, msg):
       safe = TwistStamped()
       safe.header = msg.header

       blocked = self.real_blocked or self.sim_blocked # 
       forward_requested = msg.twist.linear.x > 0.0

       self.get_logger().info(
           f"real_blocked={self.real_blocked} sim_blocked={self.sim_blocked} "
           f"real_min={self.real_min_distance:.2f} sim_min={self.sim_min_distance:.2f} "
           f"lin.x={msg.twist.linear.x:.2f} ang.z={msg.twist.angular.z:.2f}"
       )

       if blocked and forward_requested:
           safe.twist.linear.x = 0.0
           safe.twist.linear.y = 0.0
           safe.twist.linear.z = 0.0
           safe.twist.angular.x = 0.0
           safe.twist.angular.y = 0.0

           # allow turning
           safe.twist.angular.z = msg.twist.angular.z

           self.get_logger().warn("STOP: obstacle detected in real or sim front sector")
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
