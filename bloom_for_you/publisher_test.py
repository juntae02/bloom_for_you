import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import command


class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(command, 'command_topic', 10)
        timer_period = 1.0  # 초 단위
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = command()
        msg.id = self.count
        msg.command = self.count * 10
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: id={msg.id}, command={msg.command}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
