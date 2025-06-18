import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo
from bloom_for_you.function_modules import robot
from bloom_for_you.function_modules import yolo
from bloom_for_you.function_modules.tts import tts, make_txt
import DR_init

CMD_SEED = 2    # 씨앗 심기 노드 실행
FINISH_SEED = 3 # 씨앗 심기 완료

POS_SEED1 = [642.63,96.38,134.88,8.01,-180.00,7.71]         # 씨앗1 위치
POS_SEED2 = [642.64,-68.62,134.90,17.35,180.00,17.05]       # 씨앗2 위치(튤립)
POS_SEED3 = [642.62,-228.98,134.89,160.31,180.00,160.02]    # 씨앗3 위치(해바라기)

POS_TABLE = [280.28, 63.48, 187.74, 20.24, -179.96, 19.97]      # 화분 위치
POS_ZONE1 = [108.99, -461.86, 197.16, 104.45, -178.30, -168.57] # 플랜트 존 위치1(왼쪽)
POS_ZONE2 = [-114.80, -460.45, 197.16, 69.36, 180.00, 163.36]   # 플랜트 존 위치2(오른쪽)

POS_PLANT = [POS_TABLE, POS_ZONE1, POS_ZONE2, POS_SEED1, POS_SEED2, POS_SEED3]

class SeedPlanting(Node):
    def __init__(self):
        super().__init__('seed_planting')
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flower_info', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(FlowerInfo, 'flower_info', 10)
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
        
        self.robot_instance.move_home()
        self.robot_instance.close_grip()
        # tts("씨앗 심기를 시작합니다.")
        self.get_logger().info("씨앗 심기를 시작합니다.")

        self.move_seed()    # 씨앗 위치로 이동
        # self.pickup_seed()  # 씨앗 집기
        # self.plant_seed()   # 씨앗 심기
        # self.move_zone()    # 화분 이동
        # self.end_planting()   # 종료 알림


    def move_seed(self):
        if self.flower_name == "해바라기":
            self.get_logger().info('해바라기 씨앗 바구니로 이동 중...')
            self.robot_instance.move(POS_PLANT[5])
        elif self.flower_name == "튤립":
            self.get_logger().info('튤립 씨앗 바구니로 이동 중...')
            self.robot_instance.move(POS_PLANT[4])
        else:
            self.get_logger().info('원하시는 씨앗이 없습니다.')

    def pickup_seed(self):
        self.get_logger().info('씨앗 가져오는 중...')
        self.robot_instance.open_grip()
        # yolo 이동
        self.robot_instance.close_grip()
        self.robot_instance.move_relative([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])  # 위로 50 이동
        
    def plant_seed(self):
        self.get_logger().info('씨앗 운반 중...')
        self.robot_instance.move(POS_PLANT[0])
        # yolo 이동
        self.get_logger().info('씨앗 심는 중...')
        # self.robot_instance.move_relative([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])  # 그립 open 각 안 나올 때
        self.robot_instance.open_grip()

    def move_zone(self):
        self.get_logger().info('화분 운반 중...')
        # self.robot_instance.move_relative([20.0, 20.0, 0.0, 0.0, 0.0, 0.0])  # 집기 위한 위치 맞추기
        # self.robot_instance.move_relative([0.0, 0.0, -10.0, 0.0, 0.0, 0.0])  # 집기 위해 살짝 내려가기
        self.robot_instance.close_grip()
        # self.robot_instance.move_relative([0.0, 0.0, 20.0, 0.0, 0.0, 0.0])  # 살짝 위로 올라가기

        self.robot_instance.move(POS_PLANT[self.zone_number])
        # self.robot_instance.move_relative([0.0, 0.0, -30.0, 0.0, 0.0, 0.0])  # 살짝 밑으로 내려가기
        self.robot_instance.force_on_z(-10) # z축 방향 힘
        self.robot_instance.check_touch()   # 힘 감지시 정지
        self.robot_instance.open_grip()

        # 복귀
        self.get_logger().info('복귀 중...')
        self.robot_instance.move(POS_PLANT[self.zone_number])
        self.robot_instance.move_home()
        self.robot_instance.close_grip()

    def end_planting(self):
        msg = FlowerInfo()
        msg.id = self.id
        msg.command = self.command + 1
        msg.zone_number = self.zone_number
        msg.flower_name = self.flower_name
        msg.flower_meaning = self.flower_meaning
        msg.growth_duration_days = self.growth_duration_days
        msg.watering_cycle = self.watering_cycle
        msg.growth_state = 0  # 꽃 성장 상태 0

        self.cmd_pub.publish(msg)
        self.get_logger().info('전송 완료')

    
def main(args=None):
    node = SeedPlanting()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
