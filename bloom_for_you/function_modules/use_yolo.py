import rclpy
from rclpy.executors import MultiThreadedExecutor

from bloom_for_you.function_modules import yolo_
from bloom_for_you.function_modules import detection_
from bloom_for_you.function_modules import robot_control_


def main(args=None):
    if not rclpy.ok():
        rclpy.init()
    # 1. YOLO 모델 준비
    pt_path = "/home/user/my_model/best.pt"
    json_path = "/home/user/my_model/best.json"
    model = yolo_.YoloModel(
    weights_path=pt_path,
    json_path=json_path
    )
    
    # 2. 두 노드 생성
    object_node = detection_.ObjectDetectionNode(model=model)
    robot_control_node = robot_control_.RobotController()
        
    # 3. Executor 설정
    executor = MultiThreadedExecutor()
    executor.add_node(object_node)
    executor.add_node(robot_control_node)
    
    # 4. 별도 스레드 없이, Executor에서 병렬 실행
    try:
        executor.spin()
    finally:
        object_node.destroy_node()
        robot_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
