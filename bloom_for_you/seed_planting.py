import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo

CMD_SEED = 2 # 씨앗 심기 노드 실행

class FlowerSubscriber(Node):
    def __init__(self):
        super().__init__('flower_subscriber')
        self.subscription = self.create_subscription(
            FlowerInfo,
            'flower_info',
            self.flower_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def flower_callback(self, msg):
        self.get_logger().info('메시지 받았어!')
        self.get_logger().info(f"Id: {msg.id}")
        self.get_logger().info(f"Id: {msg.command}")
        self.get_logger().info(f"Flower: {msg.flower_name}")
        self.get_logger().info(f"Meaning: {msg.flower_meaning}")
        self.get_logger().info(f"Duration (days): {msg.growth_duration_days}")
        self.get_logger().info(f"Watering cycle (days): {msg.watering_cycle}")
        self.get_logger().info(f"Growth state: {msg.growth_state}")
        
        # 이 정보 토대로 해당 지점으로 이동하거나 후속 작업 수행 가능
        # self.move_to_flower_location(msg)

    def move_to_flower_location(self, msg):
        """메시지 정보에 맞추어 특정 장소로 이동하게 만드는 기능 구현"""
        self.get_logger().info('다음 지점을 향해 이동합니다.')
        # TODO: 네비게이션이나 액션서버 연결
    
def main(args=None):
    rclpy.init(args=args)
    node = FlowerSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
