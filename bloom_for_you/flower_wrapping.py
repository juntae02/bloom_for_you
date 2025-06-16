import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from bloom_for_you.function_modules.tts import tts
from bloom_for_you.function_modules.onrobot import RG
from bloom_for_you_interfaces.msg import Command
from bloom_for_you_interfaces.msg import FlowerInfo
# for single robot
# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "m0609"
# VELOCITY, ACC = 40, 40
# GRIPPER_NAME = "rg2"
# TOOLCHARGER_IP = "192.168.1.1"
# TOOLCHARGER_PORT = "502"
# DEPTH_OFFSET = -15.0
# MIN_DEPTH = 2.0

# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL

# rclpy.init()
# dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
# DR_init.__dsr__node = dsr_node

# try:
#     from DSR_ROBOT2 import movej, movel, mwait, DR_MV_MOD_REL
# except ImportError as e:
#     print(f"Error importing DSR_ROBOT2: {e}")
#     sys.exit()

# ########### Gripper Setup. Do not modify this area ############

# gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### FlowerWrapping ############

CMD_START_WRAPPING = 3
CMD_PRINT_CARD = 4

POS_ID = []
POS_BAG = []
POS_TABLE = []
POS_CARD = []

class FlowerWrapping(Node):
    def __init__(self):
        super().__init__("wrapping_node")
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flowerinfo', self.wrap_flower, 10)
        self.qr_pub = self.create_publisher(FlowerInfo,'flowerinfo',10)

    def wrap_flower(self, msg):
        self.command = msg.command
        if self.command != CMD_START_WRAPPING:
            return
        
        self.id = msg.id
        self.flower_name = msg.flower_name
        self.flower_meaning = msg.flower_meaning
        self.growth_duration_days = msg.growth_duration_days
        self.watering_cycle = msg.watering_cycle
        self.growth_state = msg.growth_state

        tts("포장을 시작합니다.")
        self.get_logger().info("포장을 시작합니다.")

        # self._ready_robot()
        self._get_box()
        
        tts('포장이 완료되었습니다.')
        self.get_logger().info("포장이 완료되었습니다.")

    # def _ready_robot(self):
    #     gripper.close_gripper()
    #     WReady = [0, 0, 90, 0, 90, 0]
    #     movej(WReady, vel=VELOCITY, acc=ACC)
    #     gripper.open_gripper()
    #     mwait()

    def _get_box(self):
        self.get_logger().info("박스/쇼핑백 가져오는 중...")

        self._get_flower()

    def _get_flower(self):
        self.get_logger().info("화분 가져오는 중...")

        self._request_card_print()

    def _request_card_print(self):
        msg = FlowerInfo()
        msg.id = self.id
        msg.command = CMD_PRINT_CARD
        msg.flower_name = self.flower_name
        msg.flower_meaning = self.flower_meaning
        msg.growth_duration_days = self.growth_duration_days
        msg.watering_cycle = self.watering_cycle
        msg.growth_state = self.growth_state
        
        self.qr_pub.publish(msg)
        self.get_logger().info("카드를 출력중입니다...")
        time.sleep(3)  # 추후: 서비스 응답 대기로 교체

        self._insert_card()
        
    def _insert_card(self):
        self.get_logger().info("카드 넣는 중...")
        

def main(args=None):
    rclpy.init(args=args)
    node = FlowerWrapping()
    try:
        rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

        
