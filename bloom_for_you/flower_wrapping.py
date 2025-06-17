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

CMD_START_WRAPPING = 20
CMD_PRINT_CARD = 21
CMD_END_WRAPPING = 22

POS_TABLE = [280.28, 63.48, 187.74, 20.24, -179.96, 19.97]
POS_ZONE1 = [108.99, -461.86, 197.16, 104.45, -178.30, -168.57]
POS_ZONE2 = [-114.80, -460.45, 197.16, 69.36, 180.00, 163.36]
POS_PLANT = [POS_TABLE, POS_ZONE1, POS_ZONE2]

POS_BAG = [423.94, -362.65, 291.63, 135.36, -171.82, 166.74]
POS_CARD = []
POS_PICKUP = []

class FlowerWrapping(Node):
    def __init__(self):
        super().__init__("wrapping_node")
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flower_info', self.wrap_flower, 10)
        self.cmd_pub = self.create_publisher(FlowerInfo, 'flower_info',qos_profile=10)
        self.card_cli = self.create_client(CardSrv,'cardsrv')
        self.robot = robot.Robot()
            

    def wrap_flower(self, msg):
        self.command = msg.command
        if self.command != CMD_START_WRAPPING:
            return
        
        self.id = msg.id
        self.zone_number = msg.zone_number
        self.flower_name = msg.flower_name
        self.flower_meaning = msg.flower_meaning
        self.growth_duration_days = msg.growth_duration_days
        self.watering_cycle = msg.watering_cycle
        self.growth_state = msg.growth_state

        tts("포장을 시작합니다.")
        self.get_logger().info("포장을 시작합니다.")

        self.robot.move_home()
        self.robot.close_grip()

        self._get_bag()
        self._get_flower()
        self._get_card()
        
        tts('포장이 완료되었습니다.')
        self.get_logger().info("포장이 완료되었습니다.")


    def _get_bag(self):
        self.get_logger().info("쇼핑백 가져오는 중...")

        self.robot.move(POS_BAG)
        self.get_logger().info("1")
        self.robot.open_grip()
        self.robot.move_relative([0,0,-60,0,0,0])
        self.robot.close_grip()
        self.get_logger().info("2")
        self.robot.move_relative([0,0,60,0,0,0])
        self.robot.move(POS_TABLE)
        self.get_logger().info("3")
        self.robot.force_on_z(-10)
        self.robot.check_touch(max=7)
        self.robot.force_off()
        self.get_logger().info("4")

        self.robot.open_grip()
        self.get_logger().info("5")
        self.robot.move_home()
        self.get_logger().info("6")

        self.get_logger().info("쇼핑백 픽업 완료")
        


    def _get_flower(self):
        self.get_logger().info("화분 가져오는 중...")
        
        self.robot.move_home()

        self.robot.move(POS_PLANT[self.zone_number])
        self.robot.open_grip()
        self.robot.move_relative([0,0,-20,0,0,0])
        self.robot.close_grip()
        self.robot.move_relative([0,0, 20,0,0,0])

        self.robot.move(POS_PLANT[0])
        self.robot.move_relative([0,0,-20,0,0,0])
        self.robot.force_on_z(-10)
        self.robot.check_touch(max=10)
        self.robot.force_off()

        self.robot.open_grip()
        self.robot.move_relative([0,0,20,0,0,0])
        self.robot.close_grip()
        self.robot.move_home()
        


    def _get_card(self):
        while not self.card_cli.wait_for_service(timeout_sec = 0.1):
            self.get_logger().warning("카드 서버 준비 안됨")

        future = self.request_card_print()
        self.get_logger().info("11111111111111111111")
        future.add_done_callback(self._card_done_callback)
        self.get_logger().info("22222222222222222222222222")
        
        
    def _insert_card(self):
        self.get_logger().info("카드 넣는 중...")
        self.robot.move_home()
        # self.robot.move(POS_CARD)
        self.robot.open_grip()
        self.robot.move_relative([20,0,0,0,0,0])
        self.robot.close_grip()
        self.robot.move_relative([-20,0,0,0,0,0])
        self.robot.move(POS_PLANT[0])
        self.robot.open_grip()
        self.get_logger().info("카드 넣기 완료")
        
        self._offer_bag()
        

    def _offer_bag(self):
        self.get_logger().info("사용자에게 꽃 전달 중...")
        self.robot.move_home()

        self.robot.move(POS_PLANT[0])
        self.robot.open_grip()
        self.robot.move_relative([0,0,-20,0,0,0])
        self.robot.close_grip()
        self.robot.move_relative([0,0,20,0,0,0])

        # self.robot.move(POS_PICKUP)
        self.robot.force_on_z(-10)
        self.robot.check_touch(max=10)
        self.robot.force_off()
        self.robot.open_grip()
        # self.robot.move(POS_PICKUP)
        self.robot.close_grip()

        self.robot.move_home()

        self.complete_wrap()
        self.get_logger().info("사용자에게 전달 완료")
        
        
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
    
    def _card_done_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {str(e)}")
            return

        self.get_logger().info(f"카드 응답 받음: {response.success}")
        if response.success:
            self.get_logger().info("카드 출력 완료됨")
            self._insert_card()
    
    def complete_wrap(self):
        msg = FlowerInfo()
        msg.id = self.id
        msg.zone_number = self.zone_number
        msg.command = CMD_END_WRAPPING
        msg.flower_name = self.flower_name
        msg.flower_meaning = self.flower_meaning
        msg.growth_duration_days = self.growth_duration_days
        msg.watering_cycle = self.watering_cycle

        self.cmd_pub.publish(msg)
        self.get_logger().info("꽃 정보 전달 완료")

        return

    


    

        

def main(args=None):
    # rclpy.init(args=args)
    node = FlowerWrapping()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

        
