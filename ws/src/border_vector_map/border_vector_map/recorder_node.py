from rclpy.node import Node



class RecorderNode(Node):
    def __init__(self):
        super.__init__('RecorderNode')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        
        
        
        
        
        
        
def main(args=None):
    rclpy.init(args=args)
node = RecorderNode()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()