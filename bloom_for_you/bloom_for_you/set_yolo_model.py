import os
import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any

from ament_index_python.packages import get_package_share_directory
from bloom_for_you_interfaces.srv import SrvDepthPosition
from bloom_for_you.function_modules.realsense_ import ImgNode
from bloom_for_you.function_modules.yolo_model_ import YoloModel

PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class SetYoloModel(Node):
    def __init__(self, model: YoloModel):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = model  # 모델 인스턴스 직접 주입
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )

    def handle_get_depth(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        rclpy.spin_once(self.img_node)

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0

        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )
from ament_index_python.packages import get_package_share_directory
import os

def main(args=None):
    if not rclpy.ok():
        rclpy.init()

    package_path = get_package_share_directory("bloom_for_you")  # 절대 경로로 반환됨

    model_path = os.path.join(package_path, 'resource', 'models_and_json', 'flower.pt')
    json_path = os.path.join(package_path, 'resource', 'models_and_json', 'flower.json')
    
    # model_path = os.path.join(package_path, 'resource', 'models_and_json', 'example_model.pt')
    # json_path = os.path.join(package_path, 'resource', 'models_and_json', 'example_json.json')
    
    print(model_path)
    print(json_path)

    # 모델 인스턴스를 직접 생성해서 전달
    model = YoloModel(pt_path=model_path, json_path=json_path)
    node = SetYoloModel(model=model)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
