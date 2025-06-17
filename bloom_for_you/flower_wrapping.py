import os
import time
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from bloom_for_you.function_modules.tts import tts
from bloom_for_you.function_modules.onrobot import RG
from bloom_for_you_interfaces.msg import FlowerInfo
from bloom_for_you_interfaces.srv import CardSrv
from bloom_for_you.function_modules import robot

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
        self.card_cli = self.create_client(CardSrv,'cardsrv',10)

        while not self.card_cli.wait_for_service(timeout_sec = 0.1):
            self.get_logger().warning("카드 서버 준비 안됨")
            

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

        self._get_box()
        
        tts('포장이 완료되었습니다.')
        self.get_logger().info("포장이 완료되었습니다.")


    def _get_box(self):
        self.get_logger().info("박스/쇼핑백 가져오는 중...")
        
        self._get_flower()


    def _get_flower(self):
        self.get_logger().info("화분 가져오는 중...")

        future = self.request_card_print()
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {str(e)}")
            return
        
        self.get_logger().info(f"Result: {response.success}")
        if response.success:
            self.get_logger().info("카드 출력이 완료되었습니다.")
            self._insert_card()
        

    def request_card_print(self):
        self.get_logger().info("카드를 출력중입니다...")
        info = FlowerInfo(
            id = self.id,
            command = CMD_PRINT_CARD,
            flower_name = self.flower_name,
            flower_meaning = self.flower_meaning,
            growth_duration_days = self.growth_duration_days,
            watering_cycle = self.watering_cycle,
            growth_state = self.growth_state
        )
        request = CardSrv.Request()
        request.flowerinfo = info
        
        future = self.card_cli.call_async(request)
        return future
        
        
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

        
