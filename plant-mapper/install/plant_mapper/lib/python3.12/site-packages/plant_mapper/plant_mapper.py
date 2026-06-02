import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import threading
import sys
import termios
import tty
import json


class PlantMapper(Node):

    def __init__(self):
        super().__init__('plant_mapper')

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Current robot pose
        self.current_x = 0.0
        self.current_y = 0.0

        self.current_qx = 0.0
        self.current_qy = 0.0
        self.current_qz = 0.0
        self.current_qw = 1.0

        # Saved plants
        self.plants = []

        self.get_logger().info(
            "\nControls:\n"
            "w = forward\n"
            "s = backward\n"
            "a = rotate left\n"
            "d = rotate right\n"
            "x = stop\n\n"
            "m = save plant\n"
            "e = export plants\n"
            "q = quit\n"
        )

        self.keyboard_thread = threading.Thread(
            target=self.keyboard_listener,
            daemon=True
        )
        self.keyboard_thread.start()

    def odom_callback(self, msg):

        pose = msg.pose.pose

        self.current_x = pose.position.x
        self.current_y = pose.position.y

        self.current_qx = pose.orientation.x
        self.current_qy = pose.orientation.y
        self.current_qz = pose.orientation.z
        self.current_qw = pose.orientation.w

    def publish_velocity(self, linear_x, angular_z):

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def save_plant(self):

        plant = {
            "x": self.current_x,
            "y": self.current_y,
            "qx": self.current_qx,
            "qy": self.current_qy,
            "qz": self.current_qz,
            "qw": self.current_qw
        }

        self.plants.append(plant)

        self.get_logger().info(
            f"Plant {len(self.plants)} saved "
            f"at ({self.current_x:.2f}, {self.current_y:.2f})"
        )

    def export_plants(self):

        with open("plants.json", "w") as f:
            json.dump(
                self.plants,
                f,
                indent=4
            )

        self.get_logger().info(
            f"Exported {len(self.plants)} plants to plants.json"
        )

    def keyboard_listener(self):

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)

            while rclpy.ok():

                key = sys.stdin.read(1)

                if key == 'w':
                    self.publish_velocity(0.2, 0.0)

                elif key == 's':
                    self.publish_velocity(-0.2, 0.0)

                elif key == 'a':
                    self.publish_velocity(0.0, 0.8)

                elif key == 'd':
                    self.publish_velocity(0.0, -0.8)

                elif key == 'x':
                    self.stop_robot()

                elif key == 'm':
                    self.save_plant()

                elif key == 'e':
                    self.export_plants()

                elif key == 'q':
                    self.stop_robot()
                    rclpy.shutdown()
                    break

        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )


def main(args=None):

    rclpy.init(args=args)

    node = PlantMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()