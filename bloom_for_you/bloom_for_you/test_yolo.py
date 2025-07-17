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
        target = input("타겟")
        x = int(input("x"))
        y = int(input("y"))
        z = int(input("z"))
        yolo_instance.grip_target(target,x, y,z)

    rclpy.shutdown()

if __name__ == '__main__':
    main()