import rclpy
from rclpy.node import Node
from bloom_for_you.function_modules import wake_up_word_
from bloom_for_you.function_modules import tts
# for mic
import pyaudio

# multi thread에서 사용
import threading

# for keyword 추출
from bloom_for_you.function_modules import keyword_extraction
from std_msgs.msg import String

"""
진행 상황
- 메시지 입력 받음
- 입력 받은 메시지 listen_command 토픽으로 퍼블리시

추가 할 일
- 추가 명령 받기
    - 명령 부족할 때 명령만 다시 받기
    - 번호 부족할 때 번호만 다시 받기
    - 다시하기를 고르면 다시하기

"""

class SpeechToCommand(Node):
    def __init__(self):
        super().__init__('speech_to_command_node')
        self.WAKE_UP_STATE = 0
        
        self.publisher = self.create_publisher(String, 'listen_command', 10)
        
        # 응답 "1234/1" 형식
        self.response = ""

        # 🧵 run_loop는 별도 스레드에서 실행
        self.loop_thread = threading.Thread(target=self.run_loop, daemon=True)
        self.loop_thread.start()

    def run_loop(self):
        while rclpy.ok():
            self.wake_up()
            self.listen_command()
            self.publish_command()

    def wake_up(self):
        if self.WAKE_UP_STATE == 0:
            self.get_logger().info("SAY HELLO ROKEY TO START!!!")
        
            # 🔊 오디오 스트림 열기
            p = pyaudio.PyAudio()
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=48000,
                input=True,
                frames_per_buffer=1024
            )

            # 🧠 Wakeword 감지 모델 초기화
            wakeup = wake_up_word_.WakeupWord(
                stream=stream,
                model_name="hello_rokey.tflite",
                buffer_size=1024
            )
            
            # while not wakeup.is_wakeup():
            #     pass
            
            input("enter to next")

            self.WAKE_UP_STATE = 1
            print("WAKE UP!!")
            tts.tts("네 안녕하세요!! 예약번호와 원하시는 명령을 말해주세요. 명령은 씨앗선택, 음성녹음, 포장이 있습니다")

        else:
            pass

    def listen_command(self):
        if self.WAKE_UP_STATE == 1:
            self.response = keyword_extraction.keyword_extraction("/home/we/rokey_ws/build/bloom_for_you/resource/get_command_prompt.txt")
            # 음성 -> 커맨드 추출 실행
            return_val = tts.make_txt("예약번호는 {}입니다. {}을 고르셨습니다 이대로 진행할까요?", [1234, "씨앗선택"])
            
            print(self.response)
            input("enter to end")
            
            tts.tts(return_val)
            self.WAKE_UP_STATE = 2
        else:
            pass        

    def publish_command(self):
        if self.WAKE_UP_STATE == 2:
            
            msg = String()
            msg.data = str(self.response)
            self.publisher.publish(msg)
            self.get_logger().info(f"🚀 Published String: {msg.data}")

            self.WAKE_UP_STATE = 0

        else:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToCommand()

    try:
        rclpy.spin(node)  # 이벤트 루프 계속 유지
    except KeyboardInterrupt:
        print("🛑 Ctrl+C pressed, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
