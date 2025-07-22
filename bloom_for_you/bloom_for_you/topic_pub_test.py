import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo

class PublisherExample(Node):
    def __init__(self):
        super().__init__('publisher_example')
        self.publisher_ = self.create_publisher(FlowerInfo, 'flower_info', 10)

    def publish_cmd_1(self):
        msg = FlowerInfo()
        msg.id = 24
        msg.command = 1  # 만약 FlowerInfo 메시지에 command 라는 int 필드가 있다면
        msg.zone_number = 2
        self.publisher_.publish(msg)
        self.get_logger().info('Published command = 1')

def main():
    rclpy.init()
    node = PublisherExample()
    node.publish_cmd_1()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()