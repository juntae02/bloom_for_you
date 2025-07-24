import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import FlowerInfo
from bloom_for_you.function_modules import robot
from bloom_for_you.function_modules import yolo
from bloom_for_you.function_modules.tts import tts
import DR_init
import time
from bloom_for_you.function_modules.onrobot_ import RG

CMD_SEED = 2     # 씨앗 심기 노드 실행
FINISH_SEED = 3  # 씨앗 심기 완료
CALL_MANAGER = 4 # 관리자 호출 번호

# ────── 재배 과정에서 필요한 좌표 ──────
 # 초기 화분 위치
POS_TABLE = [280.28, 63.48, 187.74, 20.24, -179.96, 19.97]      # 화분 위치
 # 재배하는 장소
POS_ZONE1 = [108.99, -461.86, 197.16, 104.45, -178.30, -168.57] # 플랜트 존 위치1(왼쪽)
POS_ZONE2 = [-114.80, -460.45, 197.16, 69.36, 180.00, 163.36]   # 플랜트 존 위치2(오른쪽)
 # 씨앗들 보관 장소
POS_SEED1 = [642.63,96.38,134.88,8.01,-180.00,7.71]         # 씨앗1 위치(비어둔 공간)
POS_SEED2 = [642.64,-68.62,134.90,17.35,180.00,17.05]       # 씨앗2 위치(튤립)
POS_SEED3 = [642.62,-228.98,134.89,160.31,180.00,160.02]    # 씨앗3 위치(해바라기)

POS_PLANT = [POS_TABLE, POS_ZONE1, POS_ZONE2, POS_SEED1, POS_SEED2, POS_SEED3]

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
MAX_ATTEMPTS = 3 # 최대 재시도 횟수

# ────── 씨앗 재배 클래스 ──────
class SeedPlanting(Node):
    def __init__(self):
        super().__init__('seed_planting')
        # 꽃 추천 노드로부터 "꽃의 정보" sub
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flower_info', self.cmd_callback, 10)
        # "꽃의 정보" + "꽃의 성장 상태: 0", pub to server
        self.cmd_pub = self.create_publisher(FlowerInfo, 'flower_info', 10)

        self.robot_instance = robot.Robot()
        self.yolo_instance = yolo.Yolo()
        self.gripper_instance = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    # ────── callback 함수 ──────
    def cmd_callback(self, msg):     
        self.command = msg.command
        if self.command != CMD_SEED:
            self.get_logger().info("다른 커맨드 수신")
            return
        
        # command == 2일 경우
        self.get_logger().info("2번 커맨드 수신")
        self.id = msg.id    # 식별 변호
        self.zone_number = msg.zone_number  # 플랜트 존 위치 번호
        self.flower_name = msg.flower_name  # 꽃 이름
        self.flower_meaning = msg.flower_meaning    # 꽃말
        self.growth_duration_days = msg.growth_duration_days    # 개화까지 걸리는 기간
        self.watering_cycle = msg.watering_cycle    # 관수 주기
        self.growth_state = msg.growth_state        # 꽃 성장 상태
        
        # 초기 상태 설정
        self.robot_instance.move_home()
        self.robot_instance.close_grip()
        tts("씨앗 심기를 시작합니다")
        self.get_logger().info("씨앗 심기를 시작합니다.")
        
        # seed_planting 노드의 작동 순서
        self.move_seed()    # 씨앗 보관 장소로 이동
        self.pickup_seed()  # 씨앗 집기
        self.plant_seed()   # 씨앗 심기
        self.move_zone()    # 재배하는 장소로 이동
        self.end_planting() # 노드 종료 알림

    # ────── 씨앗 보관 장소로 이동 함수 ──────
    def move_seed(self):
        # 해바라기 씨앗
        if self.flower_name == "해바라기":
            self.get_logger().info('해바라기 씨앗 바구니로 이동 중...')
            tts("해바라기 씨앗 바구니로 이동합니다")
            self.robot_instance.move(POS_PLANT[5])
        # 튤립 씨앗
        elif self.flower_name == "튤립":
            self.get_logger().info('튤립 씨앗 바구니로 이동 중...')
            tts("튤립 씨앗 바구니로 이동합니다")
            self.robot_instance.move(POS_PLANT[4])
        # 다른 씨앗의 재고가 없는 상황을 가정
        else:
            tts("요청한 씨앗의 재고가 없습니다.")
            self.call_manager()

    # ────── 씨앗 집기 함수 ──────
    def pickup_seed(self):
        self.get_logger().info('씨앗 가져오는 중...')
        tts("씨앗을 가져옵니다")

        attempt = 0
        success = False
        # 예외처리 추가: 씨앗을 집지 못했을 경우
        while attempt < MAX_ATTEMPTS and not success:
            print(f"[시도 {attempt+1}] SEED 감지 중...")
            self.robot_instance.open_grip()
            time.sleep(1.0)
            
            # YOLO로 객체 위치 감지 및 pick
            target = "씨앗"
            x = 0
            y = 10
            z = -9 
            self.yolo_instance.grip_target(target, x, y, z)
            time.sleep(7.0) # 객체를 탐지하고 이동하는 시간 고려
            self.robot_instance.close_grip()
            time.sleep(1.0)

            # 그리퍼 상태 확인
            grip_status = self.gripper_instance.get_status()[1]
            if grip_status == 1:
                print("물체를 성공적으로 집었습니다.")
                success = True
                # 현재 위치에서 z축으로 이동(상대좌표)
                self.robot_instance.move_relative([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
                time.sleep(1.0)
                    
            else:
                print("물체를 집지 못했습니다. 다시 감지 시도합니다.")
                attempt += 1
                self.robot_instance.move_relative([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
                if self.flower_name == "해바라기":
                    self.robot_instance.move(POS_PLANT[5])
                elif self.flower_name == "튤립":
                    self.robot_instance.move(POS_PLANT[4])
                time.sleep(1.0)  # 재시도 전 잠시 대기

        # 해바라기나 튤립의 씨앗 재고가 없는 상황을 가정
        if attempt == 3:
            tts("요청한 씨앗의 재고가 없습니다.")
            self.call_manager()

    # ────── 씨앗 심기 함수 ──────    
    def plant_seed(self):
        self.get_logger().info('씨앗 운반 중...')
        tts("씨앗을 운반합니다")
        self.robot_instance.move_home()
        time.sleep(1.0)

        # yolo로 화분 감지 및 이동
        self.get_logger().info('씨앗 심는 중...')
        tts("씨앗을 심습니다")
        target = "화분"
        x = -10
        y = 10
        z = 110
        self.yolo_instance.grip_target(target,x, y,z)
        time.sleep(5.0) # 객체를 탐지하고 이동하는 시간 고려
        self.robot_instance.open_grip() # 씨앗 심기

    # ────── 재배하는 장소로 이동 함수 ──────
    def move_zone(self):
        self.get_logger().info('화분 운반 중...')
        tts("화분을 운반합니다")
        # 화분을 집기 위해, 테두리 위치로 이동
        self.robot_instance.move_relative([-60.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 좌우 조절
        self.robot_instance.move_relative([0.0, 0.0, -20.0, 0.0, 0.0, 0.0])  # 높이 조절
        self.robot_instance.close_grip()
        self.robot_instance.move_relative([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])  # 높이 조절

        # 비어있는 플랜트 존으로 이동
        self.robot_instance.move(POS_PLANT[self.zone_number])
        self.robot_instance.move_relative([0.0, 0.0, -250.0, 0.0, 0.0, 0.0])  # 높이 조절
        time.sleep(1.0)

        # 바닥에 화분 놓기
        self.robot_instance.force_on_z(-10) # z축 방향 힘
        time.sleep(1.0)
        self.robot_instance.check_touch(max=14)   # 힘 감지시 정지
        time.sleep(1.0)
        self.robot_instance.open_grip()

        # 복귀
        self.get_logger().info('복귀 중...')
        self.robot_instance.move(POS_PLANT[self.zone_number])
        self.robot_instance.move_home()
        self.robot_instance.close_grip()
    
    # ────── 노드 종료 알림 함수 ──────
    def end_planting(self):
        msg = FlowerInfo()
        msg.id = self.id
        msg.command = self.command + 1  # 씨앗 심기 노드 종료 num
        msg.zone_number = self.zone_number
        msg.flower_name = self.flower_name
        msg.flower_meaning = self.flower_meaning
        msg.growth_duration_days = self.growth_duration_days
        msg.watering_cycle = self.watering_cycle
        msg.growth_state = 0  # 꽃 성장 상태 0(성장 시작)

        self.cmd_pub.publish(msg)
        self.get_logger().info('전송 완료')

    # ────── 관리자 호출 함수 ──────
    def call_manager(self):
        self.get_logger().info('관리자 호출이 필요합니다.')
        tts("관리자 호출이 필요합니다.")
        error_msg = FlowerInfo()
        error_msg.id = self.id
        error_msg.command = CALL_MANAGER  # '4'
        error_msg.zone_number = self.zone_number
        error_msg.flower_name = self.flower_name
        error_msg.flower_meaning = self.flower_meaning
        error_msg.growth_duration_days = self.growth_duration_days
        error_msg.watering_cycle = self.watering_cycle
        error_msg.growth_state = 0 

        self.cmd_pub.publish(error_msg)
        self.get_logger().info('전송 완료')

        rclpy.shutdown()

    
def main(args=None):
    node = SeedPlanting()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # shutdown 중복 호출 방지
        if rclpy.ok():
            rclpy.shutdown()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()