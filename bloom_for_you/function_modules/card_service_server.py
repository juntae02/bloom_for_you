import os
import rclpy
from rclpy.node import Node
from bloom_for_you_interfaces.srv import CardSrv
import bloom_for_you.make_letter_card as make_letter_card

class CardServiceServer(Node):
    def __init__(self):
        super().__init__('card_service_server')
        self.card_sv = self.create_service(CardSrv, 'cardsrv', self.callback)
        self.get_logger().info("카드 생성 서비스(cardsrv) 준비 완료")

    def callback(self, request, response):
        res_num = request.res_num
        self.get_logger().info(f"카드 생성 요청: 예약번호={res_num}")
        try:
            make_letter_card.make_card_service_entry(
                res_num, openai_api_key=os.getenv("OPENAI_API_KEY")
            )
            response.success = True
            self.get_logger().info(f"카드 생성 완료: 예약번호={res_num}")
        except Exception as e:
            self.get_logger().error(f"카드 생성 실패: {e}")
            response.success = False
        return response

def main():
    rclpy.init()
    node = CardServiceServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
