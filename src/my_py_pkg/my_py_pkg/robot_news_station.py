import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Robot_news_station(Node):
    def __init__(self):
        super().__init__('robot_news_station')
        self.publisher_ = self.create_publisher(String,'news',10)
        self.timer = self.create_timer(2, self.publish_message)
    def publish_message(self):
        msg = String()
        msg.data = 'This is the news!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
def main(args=None):
    rclpy.init(args=args)
    node = Robot_news_station()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()