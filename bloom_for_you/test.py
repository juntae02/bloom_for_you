import os
import rclpy
from rclpy.node import Node
from bloom_for_you.function_modules.stt import STT

openai_api_key = os.getenv("OPENAI_API_KEY")

# class Test(Node):
#     def __init__(self):
#         super().__init__('test_node')
#         self.stt = STT(openai_api_key)
#         self.stt.speech2text()


# def main():
#     rclpy.init()
#     node = Test()
#     rclpy.spin_once(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

class Test(Node):
    def __init__(self):
        super().__init__('test_node')
    
    subscriber(, callback = recommend_flower)

    def recommend_flower():
        STT.speech2text()


def main():
    rclpy.init()
    node = Test()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()