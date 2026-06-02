import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped

import threading
import sys
import termios
import tty


class ForwardSoundNode(Node):

    def __init__(self):
        super().__init__('forward_sound_node')

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.active = False

        self.move_timer = self.create_timer(
                0.1,
                self.move_robot
            )

        self.sound_timer = self.create_timer(
                3.0,
                self.play_sound
            )

        self.get_logger().info(
                "Controls:\n"
                "s = start\n"
                "p = pause\n"
                "r = resume"
            )

        self.keyboard_thread = threading.Thread(
                target=self.keyboard_listener,
                daemon=True
            )
        self.keyboard_thread.start()

    def move_robot(self):
        if not self.active:
            return

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def play_sound(self):
        if not self.active:
            return

        self.get_logger().info("BEEP!")

        # Uncomment if you have a speaker
        # os.system("aplay beep.wav")

    def stop_robot(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)

            while rclpy.ok():
                key = sys.stdin.read(1)

                if key == 's':
                    self.active = True
                    self.get_logger().info("STARTED")

                elif key == 'p':
                    self.active = False
                    self.stop_robot()
                    self.get_logger().info("PAUSED")

                elif key == 'r':
                    self.active = True
                    self.get_logger().info("RESUMED")

        finally:
            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old_settings
            )


def main(args=None):
    rclpy.init(args=args)

    node = ForwardSoundNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

