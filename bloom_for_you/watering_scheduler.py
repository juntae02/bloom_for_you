import os
import time
import datetime
import cv2
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from bloom_for_you.function_modules.tts import tts
from bloom_for_you.function_modules.onrobot import RG

from bloom_for_you.function_modules.realsense import ImgNode
from bloom_for_you.function_modules.yolo import YoloModel

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

########### Gripper Setup. Do not modify this area ############

# gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### FlowerWatering ############

current_dir = os.path.dirname(os.path.abspath(__file__))
# package_path = "/home/rokey/r2_ws/src/bloom_for_you"
package_path = os.path.abspath(os.path.join(current_dir, ".."))

CMD_START_WATERING = 5
CMD_CHECK_GROWTH_COMPLETE = 6

POS_ID = []
POS_WATER = []
POS_TABLE = []
POS_SUPPORT_ZONE = []

current_dir = os.path.dirname(os.path.abspath(__file__))
# package_path = "/home/rokey/r2_ws/src/bloom_for_you"
package_path = os.path.abspath(os.path.join(current_dir, ".."))

class FlowerWatering(Node):
    def __init__(self):
        super().__init__("watering_node")
        self.cmd_sub = self.create_subscription(FlowerInfo, 'flowerinfo', self.water_the_flower, 10)
        self.growth_pub = self.create_publisher(FlowerInfo, 'flowerinfo', 10)

        self.img_node = ImgNode()
        self.model = self._load_model('yolo')
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")
        
    def water_the_flower(self, msg):
        self.command = msg.command
        if self.command != CMD_START_WATERING:
            return
    
        self.id = msg.id
        self.flower_name = msg.flower_name
        self.flower_meaning = msg.flower_meaning
        self.growth_duration_days = msg.growth_duration_days
        self.watering_cycle = msg.watering_cycle
        self.growth_state = msg.growth_state


        tts("물 주기를 시작합니다.")
        self.get_logger().info("물 주기 시작")
        
        self._get_flower()
        
        
    def _get_flower(self):
        self.get_logger().info("화분 가져오는 중...")

        self._water()

    def _water(self):
        self.get_logger().info("물 주기 실행")

        self._check_growth()

    def _check_growth(self):
        msg = FlowerInfo()
        msg.id = self.id
        msg.command = CMD_CHECK_GROWTH_COMPLETE
        msg.flower_name = self.flower_name
        msg.flower_meaning = self.flower_meaning
        msg.growth_duration_days = self.growth_duration_days
        msg.watering_cycle = self.watering_cycle

        rclpy.spin_once(self.img_node)

        
        grow_state = 1
        self.growth_state=grow_state
        msg.growth_state = self.growth_state

        self.growth_pub.publish(msg)
        self._take_pictures()

    def _take_pictures(self):
        color_image = self.img_node.get_color_frame()
        if color_image is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"flower_{self.id}_{timestamp}.jpg"
            save_dir= os.path.join(package_path, "resource", "pictures", str(self.id))
            os.makedirs(save_dir, exist_ok=True)

            filepath = os.path.join(save_dir, filename)

            cv2.imwrite(filepath, color_image)
            self.get_logger().info(f"사진 저장 완료: {filepath}")
        else:
            self.get_logger().warn("사진 저장 실패: 이미지 프레임이 비어 있음")

        self._return_flower()

    def _return_flower(self):
        tts("물 주기를 완료하였습니다.")
        self.get_logger().info("물 주기 완료")

    

def main(args=None):
    rclpy.init(args=args)
    node = FlowerWatering()
    try:
        rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()