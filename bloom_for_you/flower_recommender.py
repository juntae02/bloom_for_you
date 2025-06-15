import os
import rclpy
from rclpy.node import Node
from dotenv import load_dotenv
from langchain.chains import LLMChain
from langchain.chat_models import ChatOpenAI
import warnings
import time
import json

from std_msgs.msg import Int32
from bloom_for_you.function_modules.keyword_extraction import keyword_extraction
from bloom_for_you.function_modules.tts import tts

from langchain.prompts import PromptTemplate

openai_api_key = os.getenv("OPENAI_API_KEY")
prompt_path = "/home/juntae02/ros2_ws/src/bloom_for_you/resource/recommender_prompt.txt"
json_path = "/home/juntae02/ros2_ws/src/bloom_for_you/resource/flower_recommendations.json"

class ExtractKeyword(Node):
    def __init__(self):
        super().__init__('extract_keyword_node')

## command를 sub 해서 해당 코드 실행하게 하기

    def extract_keyword(self):
        response = keyword_extraction(prompt_path)
        result = response.strip().split("/")
        if len(result) != 2:
            warnings.warn("The object list is more than one.")
            return None

        object, destination = result[0], result[1]
        object = object.split()
        destination = destination.split()

        # 만약 하나라도 정보없음이라면 다시 한 번 요청할 수 있도록 처리  
        if "정보없음" in object or "정보없음" in destination:
            return None

## 기념일까지의 기간이 짧을 때는 예외처리 하기  
# => 그냥 일단 시나리오 상으로 만족했다고 하기     

        return object, destination
    

    def find_flower(self, flower_data, object, destination):
        for item in flower_data:
            if item["keyword"] == object:
                for p in item["periods"]:
                    if p["period"] == destination:
                        # return p["flower_name"], p["flower_meaning"]
                        flower = {
                            "flower_name": p["flower_name"],
                            "flower_meaning": p["flower_meaning"],
                            "growth_duration_days": p["growth_duration_days"],
                            "watering_cycle": p["watering_cycle"],
                            "image_url": p["image_url"]
                        }
                        return flower
        return None, None


def main():
    rclpy.init()
    node = ExtractKeyword()

    keyword = None
    while keyword is None:
        # 여기에 로봇이 선물 목적과 남은 기간을 묻는 부분 추가 
# 꽃 키워드 추출
        keyword = node.extract_keyword()

        if keyword is None:
            print("\n목적이나 기간 정보가 빠진 듯합니다. 한 번 더 말씀해주세요.")
            time.sleep(3.0)
        else:
            print("추출 완료!", keyword)

    object = keyword[0][0]         # 축하
    destination = keyword[1][0]    # 1개월이내

    # 테스트용
    print("\n추출된 object :", object)
    print("추출된 date   :", destination)
    # tts(object) # 읽어주는 과정

# json 파일과 꽃 키워드 매칭
    with open(json_path, 'r') as f:
        flower_data = json.load(f)

    flower = node.find_flower(flower_data, object, destination)

    if flower:
        print(f"\n추천 꽃 이름: {flower['flower_name']}")         # str
        print(f"꽃말: {flower['flower_meaning']}")             # str
        print(f"성장 기간: {flower['growth_duration_days']}일") # int
        print(f"관수 주기 {flower['watering_cycle']}일")        # int
        print(f"이미지 URL: {flower['image_url']}")            # str
    else:
        print("조건과 일치되는 꽃을 찾지 못했습니다.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
