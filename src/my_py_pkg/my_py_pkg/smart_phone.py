import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class smart_phone(Node):
    def __init__(self):
        super().__init__('smart_phone')
        self.subscription = self.create_subscription(
            String, 'news', self.listener_callback, 10)
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
def main(args=None):
    rclpy.init(args=args)
    node = smart_phone()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()