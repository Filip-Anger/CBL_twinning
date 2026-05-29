import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
 
 
class MovePublisher(Node):
 
    def __init__(self):
        super().__init__('publisher_node')
 
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.publish_cmd)
 
        self.get_logger().info(
            'Publisher node started, publishing to /cmd_vel (TwistStamped)'
        )
 
    def publish_cmd(self):
        cmd = TwistStamped()
 
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
 
        cmd.twist.linear.x = 0.2
        cmd.twist.angular.z = 0.0
 
        self.publisher_.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    
    node = MovePublisher()
    
    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
