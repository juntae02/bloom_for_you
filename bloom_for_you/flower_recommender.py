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


## 추천 꽃 화면 출력 클래스
class FlowerApp(App):
    """추출된 정보나 조건과 매칭된 꽃 데이터를 GUI로 보여줌"""  

    def __init__(self, flower, node, **kwargs):
        """flower 정보는 main에서 넘깁니다.""" 
        super().__init__(**kwargs)  
        self.flower = flower 
        self.node = node
        self.redo = False   # 재선택 확인용
        # 퍼블리셔 생성
        self.publisher = self.node.create_publisher(FlowerInfo, 'flower_info', 10)

    def build(self):
        """GUI Layout 생성""" 
        # tts("꽃이 마음에 드시나요?")
        self.title = "bloom_for_you"

        layout = BoxLayout(orientation='vertical', padding=10, spacing=10)
        # 흰색 배경 추가
        with layout.canvas.before:
            Color(1, 1, 1, 1)  # 흰색
            self.rect = Rectangle(pos=layout.pos, size=layout.size)

        # 윈도우 사이즈 변경시에도 흰색 영역 변경
        layout.bind(pos=lambda inst, val: setattr(self.rect, 'pos', inst.pos),
                    size=lambda inst, val: setattr(self.rect, 'size', inst.size))
        
        # 꽃 이름
        name = Label(text=f"{self.flower['flower_name']}", markup=False, font_name=font_path, font_size='36sp', bold=True, halign='center', color=[0, 0, 0, 1], size_hint=(1, None), height=60)
        name.bind(width=lambda inst, val: setattr(inst, 'text_size', (val, inst.height)))
        layout.add_widget(name)

        # 꽃말
        meaning = Label(text=f"{self.flower['flower_meaning']}", markup=False, font_name=font_path, font_size='22sp', bold=True,halign='center', color=[1, 0, 0, 1], size_hint=(1, None), height=22, padding=[10, 0])
        meaning.bind(width=lambda inst, val: setattr(inst, 'text_size', (val - 20, inst.height)))
        layout.add_widget(meaning)

        # 사진
        img_anchor = AnchorLayout(anchor_x='center', size_hint=(1, 1))
        img = Image( source=self.flower['image_url'], size_hint=(0.7, 0.95), allow_stretch=True, keep_ratio=False)
        img_anchor.add_widget(img)
        layout.add_widget(img_anchor)
        
        # 성장 기간 
        growth_layout = BoxLayout(orientation='horizontal', size_hint=(1, None), height=30, spacing=20, padding=[20, 0])
        # growth = Label(text=f" [b]성장 기간:[/b] {self.flower['growth_duration_days']}/180", markup=True, font_name=font_path, font_size='18sp', halign='left', color=[0, 0, 0, 1], width=160, size_hint=(None, None), height=40)
        growth = Label(text=f" [b]개화 기간[/b]({self.flower['growth_duration_days']}일)", markup=True, font_name=font_path, font_size='20sp', halign='left', color=[0, 0, 0, 1], width=160, size_hint=(None, None), height=40)
        growth.bind(width=lambda inst, val: setattr(inst, 'text_size', (val - 20, inst.height)))
        growth_layout.add_widget(growth)
        bar_anchor = AnchorLayout(anchor_y='center', padding=[0, 5, 0, 0])
        growth_bar = ProgressBar(max=180, value=self.flower['growth_duration_days'], size_hint=(1, None), height=15)
        bar_anchor.add_widget(growth_bar)
        growth_layout.add_widget(bar_anchor)
        layout.add_widget(growth_layout)
        layout.add_widget(Widget(size_hint=(1, None), height=3))  # 간격 확보

        # 관수 주기
        water_layout = BoxLayout(orientation='horizontal', size_hint=(1, None), height=30, spacing=20, padding=[20, 0])
        # water = Label(text=f" [b]관수 주기:[/b] {self.flower['watering_cycle']}/7", markup=True, font_name=font_path, font_size='20sp', halign='left', color=[0, 0, 0, 1], width=160, size_hint=(None, None), height=40)
        water = Label(text=f" [b]관수 주기[/b]({self.flower['watering_cycle']}일)  ", markup=True, font_name=font_path, font_size='20sp', halign='left', color=[0, 0, 0, 1], width=160, size_hint=(None, None), height=40)
        water.bind(width=lambda inst, val: setattr(inst, 'text_size', (val - 20, inst.height)))
        water_layout.add_widget(water)
        water_bar_anchor = AnchorLayout(anchor_y='center', padding=[0, 5, 0, 0])
        water_bar = ProgressBar(max=7, value=self.flower['watering_cycle'], size_hint=(1, None), height=15)
        water_bar_anchor.add_widget(water_bar)
        water_layout.add_widget(water_bar_anchor)
        layout.add_widget(water_layout)
        layout.add_widget(Widget(size_hint=(1, None), height=5))  # 간격 확보
        
        # 버튼
        button_box = BoxLayout(orientation='horizontal', size_hint=(1, None), height=50, spacing=15, padding=[10, 0])
        select = Button(text='선 택', font_name=font_path, color=[0, 0, 0, 1], font_size='25sp', bold=True, background_normal='', background_color=[0.9, 0.9, 0.9, 1])
        select.bind(on_press=self.on_select)  
        button_box.add_widget(select)
        reselect = Button(text='재 선 택', font_name=font_path, color=[0, 0, 0, 1], font_size='25sp', bold=True, background_normal='', background_color=[0.9, 0.9, 0.9, 1])
        reselect.bind(on_press=self.on_reselect)  
        button_box.add_widget(reselect)
        layout.add_widget(button_box)

        return layout
    
## 재선택시, 추가적인 정보를 더 물어보는 것도 ㄱㅊ, 
# but 시나리오 상에 다운로드 받을 이미지가 많아짐
    def on_select(self, instance):
        print("선택되었습니다.")
        tts("선택되었습니다")

        msg = FlowerInfo()
        msg.id = self.flower['id']  
        msg.command = self.flower['command'] + 1
        msg.zone_number = self.flower['zone_number']
        msg.flower_name = self.flower['flower_name']
        msg.flower_meaning = self.flower['flower_meaning']
        msg.growth_duration_days = self.flower['growth_duration_days']
        msg.watering_cycle = self.flower['watering_cycle']
        msg.growth_state = 0  # 꽃 성장 상태 0

        self.publisher.publish(msg)
        self.node.get_logger().info('전송 완료')
        time.sleep(1.5)

        self.redo = False
        App.get_running_app().stop()

    def on_reselect(self, instance):
        print("재선택되었습니다.")
        tts("재선택되었습니다")
        self.redo = True
        App.get_running_app().stop()


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
                app = FlowerApp(flower, node)
                app.run()
                if app.redo:    # 재선택
                    node.get_logger().info("다시 선택합니다.")
                    continue    
                else:           # 꽃 선택
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