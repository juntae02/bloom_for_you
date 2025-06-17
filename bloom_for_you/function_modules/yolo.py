import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import GripTarget

class Yolo(Node):
    def __init__(self):
        super().__init__('grip_target_publisher')
        self.publisher_ = self.create_publisher(GripTarget, 'grip_target', 10)

    def grip_target(self, target, x=0, y=0, z=0):
        msg = GripTarget()
        msg.target = target
        msg.offset_x = x
        msg.offset_y = y
        msg.offset_z = z
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    use_yolo = Yolo()
    import time
    while (1):
        target_name = input("타겟")
        x = int(input("x 값"))
        y = int(input("y 값"))
        z = int(input("z 값"))
        use_yolo.grip_target(target_name, x, y, z)

    use_yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()