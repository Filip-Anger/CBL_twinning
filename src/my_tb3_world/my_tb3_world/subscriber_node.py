import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
 
 
class SubscriberNode(Node):
 
    def __init__(self):
        super().__init__('subscriber_node')
 
        self.subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
 
        self.get_logger().info('Subscriber node started, listening on /scan')
 
    def scan_callback(self, msg):
        n = len(msg.ranges)
        front = msg.ranges[n // 2]
 
        self.get_logger().info(
            f'LaserScan: {n} points, front = {front:.2f} m'
        )
 
 
def main(args=None):
    rclpy.init(args=args)
    
    node = SubscriberNode()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
