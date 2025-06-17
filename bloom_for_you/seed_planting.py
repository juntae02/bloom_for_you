import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo
from bloom_for_you.function_modules import robot
from bloom_for_you.function_modules.tts import tts, make_txt
import DR_init

CMD_SEED = 2    # 씨앗 심기 노드 실행
FINISH_SEED = 3 # 씨앗 심기 완료
# 씨앗1
seed1_posx = [642.63,96.38,134.88,8.01,-180.00,7.71]
# 씨앗2
seed2_posx = [642.64,-68.62,134.90,17.35,180.00,17.05]
# 씨앗3
seed3_posx = [642.62,-228.98,134.89,160.31,180.00,160.02]

class SeedPlanting(Node):
    def __init__(self):
        super().__init__('seed_planting')
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flower_info', self.cmd_callback, 10)
        # self.cmd_sub
        self.robot_instance = robot.Robot()
    
    def cmd_callback(self, msg):
        self.command = msg.command
        if self.command != CMD_SEED:
            return
        self.id = msg.id
        self.zone_number = msg.zone_number
        self.flower_name = msg.flower_name
        self.flower_meaning = msg.flower_meaning
        self.growth_duration_days = msg.growth_duration_days
        self.watering_cycle = msg.watering_cycle
        self.growth_state = msg.growth_state
        
        # self.robot_instance.move_home()
        # tts("씨앗 심기를 시작합니다.")
        self.get_logger().info("씨앗 심기를 시작합니다.")

        self.move_seed()    # 씨앗 위치로 이동
        self.pickup_seed()  # 씨앗 집기


    def move_seed(self):
        if self.flower_name == "해바라기":
            self.get_logger().info('해바라기 씨앗 바구니로 이동 중...')
            # self.robot_instance.move(seed3_posx)
        elif self.flower_name == "튤립":
            self.get_logger().info('튤립 씨앗 바구니로 이동 중...')
            # self.robot_instance.move(seed2_posx)
        else:
            self.get_logger().info('원하시는 씨앗이 없습니다.')

    def pickup_seed(self):
        self.get_logger().info('씨앗 가져오는 중...')
        # self.robot_instance.open_grip()
        # yolo 이동
        # self.robot_instance.close_grip()
        # self.robot_instance.move_relative([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])  # 위로 50 이동
        # 화분으로 이동
        # yolo
        # 화분 집어서 zone_number 위치로 이동
       

    
def main(args=None):
    node = SeedPlanting()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
