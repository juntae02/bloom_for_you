import os
import rclpy
from rclpy.node import Node
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
import warnings
import time
import json
import threading

from langchain.prompts import PromptTemplate
from std_msgs.msg import Int32
from bloom_for_you.function_modules.keyword_extraction import keyword_extraction
from bloom_for_you.function_modules.tts import tts
from bloom_for_you_interfaces.msg import FlowerInfo


from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.anchorlayout import AnchorLayout
from kivy.graphics import Color, Rectangle
from kivy.uix.progressbar import ProgressBar
from kivy.uix.widget import Widget

openai_api_key = os.getenv("OPENAI_API_KEY")
current_dir = os.getcwd()
prompt_path = current_dir + "/src/bloom_for_you/resource/recommender_prompt.txt"
json_path = current_dir + "/src/bloom_for_you/resource/flower_recommendations.json"
font_path = current_dir + "/src/bloom_for_you/resource/font/NanumGothic-Regular.ttf"

CMD_RMD = 1 # 꽃 추천 노드 실행

## 꽃 추천 클래스
class ExtractKeyword(Node):
    def __init__(self):
        super().__init__('extract_keyword_node')
        self.cmd_received = threading.Event()
        self.subscription = self.create_subscription(FlowerInfo, 'flower_info', self.cmd_callback, 10)
        self.msg_id = None
        self.msg_cmd = None

    def cmd_callback(self, msg):
        if msg.command == CMD_RMD:
            self.get_logger().info('Command 1 received, triggering main.')
            self.msg_id = msg.id
            self.msg_cmd = msg.command
            self.msg_zone = msg.zone_number
            self.cmd_received.set()

    def extract_keyword(self):
        response = keyword_extraction(prompt_path)
        result = response.strip().split("/")
        if len(result) != 2:
            warnings.warn("The object list is more than one.")
            return None

        object, destination = result[0], result[1]
        object = object.split()
        destination = destination.split()

        # 예외처리
        if "정보없음" in object or "정보없음" in destination:
            return None   
        
        return object, destination
    
## 기념일까지의 기간이 짧을 때는 예외처리 하기  
# => 그냥 일단 시나리오 상으로 만족했다고 하기  

    def find_flower(self, flower_data, object, destination):
        for item in flower_data:
            if item["keyword"] == object:
                for p in item["periods"]:
                    if p["period"] == destination:
                        flower = {
                            "id": self.msg_id,
                            "command": self.msg_cmd,
                            "zone_number": self.msg_zone,
                            "flower_name": p["flower_name"],
                            "flower_meaning": p["flower_meaning"],
                            "growth_duration_days": p["growth_duration_days"],
                            "watering_cycle": p["watering_cycle"],
                            "image_url": p["image_url"]
                        }
                        return flower
        return None


class TerminalFlowerApp:
    def __init__(self, flower, node):
        self.flower = flower
        self.node = node
        self.redo = False
        self.publisher = self.node.create_publisher(FlowerInfo, 'flower_info', 10)

    def run(self):
        # 꽃 정보 출력
        print("\n🌸 추천된 꽃 정보:")
        print(f"꽃 이름         : {self.flower['flower_name']}")
        print(f"꽃말            : {self.flower['flower_meaning']}")
        print(f"개화 기간       : {self.flower['growth_duration_days']}일 (기준: 180일)")
        print(f"관수 주기       : {self.flower['watering_cycle']}일 (기준: 7일)")
        print(f"이미지 링크     : {self.flower['image_url']}\n")

        tts("추천된 꽃이 마음에 드시나요? 마음에 드시면 1번을, 다시 추천을 원하시면 2번을 눌러주세요.")

        while True:
            try:
                choice = input("1: 선택    2: 재선택\n입력하세요: ").strip()
                if choice == "1":
                    self.on_select()
                    break
                elif choice == "2":
                    self.on_reselect()
                    break
                else:
                    print("⚠️  잘못된 입력입니다. 1 또는 2를 입력해주세요.\n")
            except KeyboardInterrupt:
                print("\n종료합니다.")
                break

    def on_select(self):
        print("✅ 선택되었습니다.")
        tts("선택되었습니다")

        msg = FlowerInfo()
        msg.id = self.flower['id']
        msg.command = self.flower['command'] + 1
        msg.zone_number = self.flower['zone_number']
        msg.flower_name = self.flower['flower_name']
        msg.flower_meaning = self.flower['flower_meaning']
        msg.growth_duration_days = self.flower['growth_duration_days']
        msg.watering_cycle = self.flower['watering_cycle']
        msg.growth_state = 0

        self.publisher.publish(msg)
        self.node.get_logger().info("꽃 선택 정보 전송 완료")
        time.sleep(1.0)
        self.redo = False

    def on_reselect(self):
        print("🔁 재선택합니다.")
        tts("재선택되었습니다")
        self.redo = True


def run_flower_logic(node):
    
    try:
        # json 파일과 꽃 키워드 매칭
        with open(json_path, 'r') as f:
            flower_data = json.load(f)
        while True:
        # 꽃 키워드 추출
            keyword = None
            while keyword is None:
## 여기에 로봇이 선물 목적과 남은 기간을 묻는 부분 추가 
                tts("선물의 목적과 남은 기간을 말씀해주세요")
                keyword = node.extract_keyword()  
                if keyword is None:
                    node.get_logger().info("\n목적이나 기간 정보가 빠진 듯합니다.\n")
                    tts("목적이나 기간 정보가 빠진 거 같습니다. 다시 한 번 말씀해주세요")
                    time.sleep(3.0)
                else:
                    node.get_logger().info(f"\n목적-{keyword[0][0]}, 기간-{keyword[1][0]}\n") 


            object = keyword[0][0]         # 축하
            destination = keyword[1][0]    # 1개월이내
            # 해바라기: 졸업 1개월이내
            # 튤립 : 축하 4-6개월

            # template:str, input_list:list
            # template = "{}, {} 2개의 키워드를 추출하였습니다"
            # input_list = [object, destination]
            # text = make_txt(template, input_list)
            # tts(text)

        

            flower = node.find_flower(flower_data, object, destination)

            if flower:
                app = TerminalFlowerApp(flower, node)
                app.run()
                if app.redo:
                    node.get_logger().info("다시 선택합니다.")
                    continue
                else:
                    node.get_logger().info("선택 완료.")
                    break
            else:
                node.get_logger().error("GUI를 실행시키지 못했습니다.")
                break
    except Exception as e:
        node.get_logger().error(f"Error in main flow: {e}")
    finally:
        # node.destroy_node()
        # rclpy.shutdown()
        pass

def main():
    rclpy.init()
    node = ExtractKeyword()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            node.get_logger().info('Waiting for CMD 1...')
            node.cmd_received.clear() 
            node.cmd_received.wait()
            node.get_logger().info('Command 1 received, now starting main flow')

            # cmd_received 발생 후 run_flower_logic 수행
            run_flower_logic(node)
    except KeyboardInterrupt:
        print("사용자에 의해 종료되었습니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join()