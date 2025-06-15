import os
import rclpy
from rclpy.node import Node
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
import warnings
import time
import json

from langchain.prompts import PromptTemplate
from std_msgs.msg import Int32
from bloom_for_you.function_modules.keyword_extraction import keyword_extraction
from bloom_for_you.function_modules.tts import tts
from bloom_for_you_interfaces.msg import Command, FlowerInfo

from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.anchorlayout import AnchorLayout

openai_api_key = os.getenv("OPENAI_API_KEY")
prompt_path = "/home/juntae02/ros2_ws/src/bloom_for_you/resource/recommender_prompt.txt"
json_path = "/home/juntae02/ros2_ws/src/bloom_for_you/resource/flower_recommendations.json"
font_path = "/home/juntae02/ros2_ws/src/bloom_for_you/resource/font/NanumGothic-Regular.ttf"


## 꽃 추천 클래스
class ExtractKeyword(Node):
    def __init__(self):
        super().__init__('extract_keyword_node')

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
        # 퍼블리셔 생성
        self.publisher = self.node.create_publisher(FlowerInfo, 'flower_info', 10)

    def build(self):
        """GUI Layout 생성""" 
        layout = BoxLayout(orientation='vertical', padding=10, spacing=10)

        # 사진
        img_anchor = AnchorLayout(anchor_x='center', size_hint=(1, 0.7))
        img = Image( source=self.flower['image_url'], size_hint=(0.65, 0.8), allow_stretch=True, keep_ratio=False)
        img_anchor.add_widget(img)
        layout.add_widget(img_anchor)

        # 꽃 이름
        name = Label(text=f"[b]꽃 이름:[/b] {self.flower['flower_name']}", markup=True, font_name=font_path, font_size='18sp', halign='left', size_hint=(1, None), height=30)
        name.bind(width=lambda inst, val: setattr(inst, 'text_size', (val, inst.height)))
        layout.add_widget(name)

        # 꽃말
        meaning = Label(text=f"[b]꽃말:[/b] {self.flower['flower_meaning']}", markup=True, font_name=font_path, font_size='18sp', halign='left', size_hint=(1, None), height=30)
        meaning.bind(width=lambda inst, val: setattr(inst, 'text_size', (val, inst.height)))
        layout.add_widget(meaning)

        # 성장 기간 
        growth = Label(text=f"[b]성장 기간:[/b] {self.flower['growth_duration_days']}일", markup=True, font_name=font_path, font_size='18sp', halign='left', size_hint=(1, None), height=30)
        growth.bind(width=lambda inst, val: setattr(inst, 'text_size', (val, inst.height)))
        layout.add_widget(growth)

        # 관수 주기
        water = Label(text=f"[b]관수 주기:[/b] {self.flower['watering_cycle']}일", markup=True, font_name=font_path, font_size='18sp', halign='left', size_hint=(1, None), height=30)
        water.bind(width=lambda inst, val: setattr(inst, 'text_size', (val, inst.height)))
        layout.add_widget(water)

        # 버튼들 (선택, 재선택)을 한 줄에 넣죠 
        button_box = BoxLayout(orientation='horizontal', size_hint=(1, None), height=50)

        select = Button(text='선택', font_name=font_path)
        select.bind(on_press=self.on_select)  
        button_box.add_widget(select)

        reselect = Button(text='재선택', font_name=font_path)
        reselect.bind(on_press=self.on_reselect)  
        button_box.add_widget(reselect)

        layout.add_widget(button_box)

        return layout
    
## 재선택시, 추가적인 정보를 더 물어보는 것도 ㄱㅊ, but 시나리오 상에 다운로드 받을 이미지가 많아짐
    def on_select(self, instance):
        print("선택되었습니다.")

        msg = FlowerInfo()
        msg.id = 123  # sub 받으면 수정
        msg.flower_name = self.flower['flower_name']
        msg.flower_meaning = self.flower['flower_meaning']
        msg.growth_duration_days = self.flower['growth_duration_days']
        msg.watering_cycle = self.flower['watering_cycle']
        msg.growth_state = 0  # 꽃 성장 상태 0

        self.publisher.publish(msg)
        self.node.get_logger().info('전송 완료')

        App.get_running_app().stop()

    def on_reselect(self, instance):
        """재선택 버튼 클릭시 다시 선택 프로세스를 수행할 수 있습니다."""  # 추가
        print("재선택되었습니다.")
        # TODO: rclpy나 다른 기능과 연결 가능


def main():
## command를 sub 해서 해당 코드 실행하게 하기

    rclpy.init()
    node = ExtractKeyword()

    keyword = None
    while keyword is None:
## 여기에 로봇이 선물 목적과 남은 기간을 묻는 부분 추가 
        keyword = node.extract_keyword()    # 꽃 키워드 추출

        if keyword is None:
            print("\n목적이나 기간 정보가 빠진 듯합니다. 한 번 더 말씀해주세요.")
            time.sleep(3.0)
        else:
            print("추출 완료!", keyword)

    object = keyword[0][0]         # 축하
    destination = keyword[1][0]    # 1개월이내
    # tts(object) # 읽어주는 과정

# json 파일과 꽃 키워드 매칭
    with open(json_path, 'r') as f:
        flower_data = json.load(f)

    flower = node.find_flower(flower_data, object, destination)

    if flower:
        app = FlowerApp(flower, node)
        app.run()
    else:
        print("GUI를 실행시키지 못했습니다.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
