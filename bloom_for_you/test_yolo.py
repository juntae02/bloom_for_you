import rclpy
from rclpy.node import Node
from bloom_for_you.function_modules import yolo
from bloom_for_you.function_modules import robot

def main(args=None):
    # rclpy.init(args=args)
    
    yolo_instance = yolo.Yolo()
    robot_instance = robot.Robot()
    while (1):
        robot_instance.move_home()
        x = input("타겟")
        y = int(input("높이"))
        yolo_instance.grip_target(x, y)

    rclpy.shutdown()

if __name__ == '__main__':
    main()