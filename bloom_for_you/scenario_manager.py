import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo
from std_msgs.msg import String # 음성으로 받은 명령어

class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager_node')
        self.ROBOT_STATE = 0
        self.i = 0
        self.started = False

        # ROS 퍼블리셔 및 서브스크라이버
        self.publisher = self.create_publisher(FlowerInfo, 'flower_info', 10)
        self.subscriber = self.create_subscription(
            FlowerInfo,
            'flower_info',
            self.flower_info_callback,
            10
        )

    def flower_info_callback(self, msg: FlowerInfo):
        self.get_logger().info(
            f"[Subscribe] id: {msg.id}, command: {msg.command}, zone: {msg.zone_number}, "
            f"name: {msg.flower_name}, meaning: {msg.flower_meaning}, "
            f"duration: {msg.growth_duration_days}d, cycle: {msg.watering_cycle}d, "
            f"state: {msg.growth_state}"
        )

def main(args=None):
    rclpy.init(args=args)  # ROS 초기화
    scenario_manager = ScenarioManager()  # 노드 생성

    try:
        rclpy.spin(scenario_manager)  # 노드 실행 (콜백 대기)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        scenario_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
