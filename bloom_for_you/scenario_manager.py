import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo
from std_msgs.msg import String # 음성으로 받은 명령어

class ScenarioManager(Node):
    def __init__(self):
        super().__init__('scenario_manager_node')
        
        self.command_queue = []
        self.ROBOT_STATE = 0
        self.LISTEN_COMMAND_STATE = 0
        self.i = 0
        self.started = False

        # ROS 퍼블리셔 및 서브스크라이버
        self.publisher = self.create_publisher(FlowerInfo, 'flower_info', 10)
        self.command_subscriber = self.create_subscription(String,'listen_command',self.listen_command_callback,10)
        self.flower_info_subscriber = self.create_subscription(FlowerInfo,'flower_info',self.flower_info_callback,10)
        
        self.run_command = self.create_timer(1.0, self.run_command)
        
    def run_command(self):
        if len(self.command_queue) != 0:
            input("enter to start run command")
            self.get_logger().info(f"run command!! number = {self.command_queue[0][0]} command = {self.command_queue[0][1]}")
            self.command_queue.pop(0)

    # 음성인식 받아서 저장
    def listen_command_callback(self, msg: String):
        self.get_logger().info(f"[Subscribe] id: {msg.data}")
        res_num, cmd_num = msg.data.split('/')
        if(res_num == "0000" or cmd_num == "0"):
            self.get_logger().warn(f"잘못된 데이터가 입력되었습니다!! 데이터: {msg.data}")
            pass
        else:
            self.command_queue.append((res_num, cmd_num))
            print("cmd list:")
      
    # 꽃 관련 커맨드 받으면 해당 내용 처리
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
