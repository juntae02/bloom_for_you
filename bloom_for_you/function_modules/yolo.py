import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.msg import GripTarget

class Yolo(Node):
    def __init__(self):
        super().__init__('grip_target_publisher')
        self.publisher_ = self.create_publisher(GripTarget, 'grip_target', 10)

    def grip_target(self, target, height):
        msg = GripTarget()
        msg.target = target
        msg.height_z = height
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    use_yolo = Yolo()
    import time
    while (1):
        x = input("타겟")
        y = int(input("높이"))
        use_yolo.grip_target(x,y)

    use_yolo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()